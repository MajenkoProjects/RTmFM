#include <CLI.h>
#include <SdFat.h>
#include "datastream.h"
#include <errno.h>
#include <pico/platform.h>


CLIClient *console;

#define SDIO_CLK	16
#define SDIO_CMD	17
#define SDIO_DAT0	18
#define SDIO_DAT1	19
#define SDIO_DAT2	20
#define SDIO_DAT3	21

#define SECTOR_128 0
#define SECTOR_256 1
#define SECTOR_512 2
#define SECTOR_1024 3


#define INDEX			12
#define DOUT			0

#define STEP			7
#define DIR				8
#define SEEK_DONE		9
#define TRACK0			10

#define HSEL0			4
#define HSEL1			5
#define HSEL2			6
#define HSEL3			7

static PIOProgram datastreamPgm(&datastream_program);

PIO pio;
int sm;
int offset;
int dma;

uint32_t loadtime;

volatile uint32_t current_head = 0;
volatile uint32_t current_cyl = 0;
volatile uint32_t current_sector = 0;
volatile uint32_t target_cyl = 0;

#define POLY16 0x1021
#define POLY32 0xa00805

#define NUM_SECTORS 17

#define TRACK_SIZE (512 * NUM_SECTORS)

#define OPT_HEADER_CRC16	0x00000001
#define OPT_HEADER_CRC32	0x00000002
#define OPT_HEADER_CRC_MASK 0xFFFFFFFC
#define OPT_DATA_CRC16		0x00000004
#define OPT_DATA_CRC32		0x00000008
#define OPT_DATA_CRC_MASK 	0xFFFFFFF3

SdFs sd;
FsFile mounted_file;
uint8_t *track_buffer;

volatile bool run_mfm_sm = false;
mutex mfm_sm_running;

struct disk_format {
	char *name;
	char *comment;
	uint16_t cyls;
	uint8_t heads;
	uint8_t sectors;
	uint8_t sector_size;
	uint32_t index_width;
	uint32_t track_pregap;
	uint32_t track_postgap;
	uint32_t header_postgap;
	uint32_t data_postgap;
	float data_rate;
	uint32_t flags;
	uint32_t header_poly;
	uint32_t data_poly;
	bool auto_return;

	// Calculated data - not to be filled.

	uint32_t tlen;
	uint32_t slen;
	uint32_t idx_ts;
	uint32_t idx_period;
	float clock_div;
};

struct format_list {
	struct disk_format *fmt;
	struct format_list *next;
};

struct format_list *formats = NULL;

struct encoded_sector {
	uint16_t header[10];
	uint16_t data[520];
};

struct raw_sector {
	uint16_t header_crc;
	uint32_t data_crc;
	uint8_t *data;
};

struct track {
	struct raw_sector *sector;
};

struct cylinder {
	uint32_t heads;
	uint32_t sectors;
	struct track *track;
};


struct cylinder cyl_data;



struct disk_format *format;

bool _sd_is_open = false;
bool open_sd_if_needed() {
	if (_sd_is_open) return true;
	if (!sd.begin(SdioConfig(SDIO_CLK, SDIO_CMD, SDIO_DAT0, 1.5))) {
		sd.initErrorPrint();
		return false;;
	}
	_sd_is_open = true;
	return true;
}

void bindump(uint16_t v) {
	for (int i = 0; i < 16; i++) {
		v <<= 1;
	}
}

uint16_t crc16(uint8_t val, uint16_t crc, uint16_t poly)
{

	uint16_t xval = val;

   int j;
   crc = crc ^ (xval << 8);
   for (j = 1; j <= 8; j++) {
      if (crc & 0x8000) {
         crc = (crc << 1) ^ poly;
      } else {
         crc = crc << 1;
      }
   }
   return crc;
}

static uint32_t CRCTable[256];

static void CRC32_init(uint32_t poly) {
	uint32_t mask = 0xFFFFFFFF;
	uint32_t topbit = 0x80000000;

	for (int i = 0; i < 256; i++) {
		uint32_t c = (i << 24);
		for (int j = 0; j < 8; j++) {
			if (c & topbit) {
				c = ((c << 1) ^ poly) & mask;
			} else {
				c = (c << 1) & mask;
			}
		}
		CRCTable[i] = c;
	}
}
static inline uint32_t crc32(uint8_t val, uint32_t crc, uint32_t poly) {
	const uint8_t idx = ((crc >> 24) ^ val) & 0xFF;
	crc = ((crc << 8) ^ CRCTable[idx]);
	return crc;
}


uint8_t last_bit = 0;
uint16_t mfm_encode_bit(uint8_t b) {
	if (b == 0x80) {
		last_bit = 1;
		return 0b01;
	}


	if (last_bit == 0) {
		return 0b10;
	} else {
		last_bit = 0;
		return 0b00;
	}
}

uint16_t mfm_encode(uint8_t b, bool reset = false) {
	if (reset) {
		last_bit = 0;
	}

	uint16_t out = 0;

	for (int i = 0; i < 8; i++) {
		out <<= 2;
		out |= mfm_encode_bit(b & 0x80);
		b <<= 1;
	}

	return out;
}

enum {
	PH_INDEX_START,
	PH_INDEX_END,
	PH_TRACK_PREGAP,
	PH_SEND_HEADER,
	PH_HEADER_POSTGAP,
	PH_SEND_DATA,
	PH_DATA_POSTGAP,
	PH_TRACK_POSTGAP
};

enum {
	LOAD_IDLE,
	LOAD_HEADER,
	LOAD_DATA,
	LOAD_DATA_RUN,
	LOAD_HEADER_CS,
	LOAD_DATA_CS,
	LOAD_HEADER_MFM,
	LOAD_DATA_MFM,
	LOAD_SYNC
};

void second_cpu_thread() {

	dma = dma_claim_unused_channel(true);
	dma_channel_config config = dma_channel_get_default_config(dma);
	channel_config_set_read_increment(&config, true);
	channel_config_set_write_increment(&config, false);
	channel_config_set_dreq(&config, pio_get_dreq(pio, sm, true));
	channel_config_set_transfer_data_size(&config, DMA_SIZE_16);

	uint32_t ts = micros();

	int phase = 0;
	int load_sm = LOAD_IDLE;
	int load_iter = 0;
	uint16_t load_hcs = 0;
	uint32_t load_dcs = 0;

	int next_sector = 1;
	current_sector = 0;

	struct encoded_sector sectorA;
	struct encoded_sector sectorB;

	struct encoded_sector *load = &sectorA;
	struct encoded_sector *send = &sectorB;
	struct encoded_sector *swap;

	pinMode(HSEL0, INPUT);
	pinMode(HSEL1, INPUT);
	pinMode(HSEL2, INPUT);
	pinMode(HSEL3, INPUT);

	uint32_t cs = current_cyl;

	bool running = false;

	while (1) {

		if (!run_mfm_sm) {
			phase = 0;
			load_sm = LOAD_IDLE;
			if (running) {
				Serial.println("Stopping state machine");
				dma_channel_wait_for_finish_blocking(dma);
				mutex_exit(&mfm_sm_running);
				Serial.println("State machine stopped");
			}
			running = false;
			continue;
		}

		if (!running) {
			Serial.println("Starting state machine");
			running = true;
			mutex_enter_blocking(&mfm_sm_running);
			Serial.println("State machine started");
		}

	    uint16_t hp = format->header_poly;

		current_head = digitalRead(HSEL0) | (digitalRead(HSEL1) << 1) | (digitalRead(HSEL2) << 2) | (digitalRead(HSEL3) << 3);

		switch (phase) {
			case PH_INDEX_START: // Start index pulse
				cs = current_cyl;
				ts = micros();
				format->idx_period = micros() - format->idx_ts;
				format->idx_ts = micros();
				digitalWrite(INDEX, LOW);
				phase++;
				break;

			case PH_INDEX_END: // Index pulse timing
				if ((micros() - ts) >= format->index_width) {
					ts = micros();
					digitalWrite(INDEX, HIGH);
					phase++;
				}
				break;

			case PH_TRACK_PREGAP: // Track pre-gap
				if ((micros() - ts) >= format->track_pregap) {
					ts = micros();
					phase = PH_SEND_HEADER;
				}
				break;

			case PH_SEND_HEADER:
				ts = micros();

				swap = load;
				load = send;
				send = swap;

				dma_channel_configure(dma, &config, 
					&pio->txf[sm],
					send->header,
					10,
					true
				);

				next_sector = (current_sector + 1) % format->sectors;
				load_sm = LOAD_HEADER;


				phase = PH_HEADER_POSTGAP;
				break;

			case PH_HEADER_POSTGAP:
				if ((micros() - ts) >= format->header_postgap) {
					ts = micros();
					phase = PH_SEND_DATA;
				}
				break;

			case PH_SEND_DATA:
				ts = micros();

				dma_channel_configure(dma, &config,
					&pio->txf[sm],
					send->data,
					520,
					true
				);
	
				if (current_sector < format->sectors - 1) {
					phase = PH_DATA_POSTGAP;
				} else {
					phase = PH_TRACK_POSTGAP;
				}
				break;

			case PH_DATA_POSTGAP:
				if ((micros() - ts) > format->data_postgap) {
					ts = micros();
					current_sector++;
					phase = PH_SEND_HEADER;
				}
				break;

			case PH_TRACK_POSTGAP:
				if ((micros() - ts) > format->track_postgap) {
					ts = micros();
					current_sector = 0;
					phase = PH_INDEX_START;
				}
				break;
		}



		// Sector loading state machine

		switch (load_sm) {
			case LOAD_IDLE: // Waiting for instruction
				break;

			case LOAD_HEADER:
					//digitalWrite(INDEX, LOW);
					//digitalWrite(INDEX, HIGH);
				load->header[0] = 0;
				load->header[1] = 0xA1;
				load->header[2] = 0xFE;
				load->header[3] = (cs & 0xFF);
				load->header[4] = ((cs & 0xF00) >> 4) | (current_head & 0x0F);
				load->header[5] = next_sector;
				load->header[6] = format->sector_size;
				load->header[7] = 0x00;
				load->header[8] = 0x00;
				load->header[9] = 0;
				load_sm = LOAD_DATA;
				break;

			case LOAD_DATA:
				load->data[0] = 0;
				load->data[1] = 0xA1;
				load->data[2] = 0xFB;
				load->data[515] = ((cyl_data.track[cs].sector[next_sector].data_crc >> 24) & 0xFF);
				load->data[516] = ((cyl_data.track[cs].sector[next_sector].data_crc >> 16) & 0xFF);
				load->data[517] = ((cyl_data.track[cs].sector[next_sector].data_crc >> 8) & 0xFF);
				load->data[518] = (cyl_data.track[cs].sector[next_sector].data_crc & 0xFF);
				load->data[519] = 0;
				load_iter = 0;
				load_sm = LOAD_HEADER_CS;
				load_iter = 1;
				load_hcs = 0xFFFF;
				break;

			case LOAD_HEADER_CS:
				load_hcs = crc16(load->header[load_iter], load_hcs, hp);
				load_iter++;
				if (load_iter == 7) {
					load->header[7] = (load_hcs >> 8) & 0xFF;
					load->header[8] = load_hcs & 0xFF;
					load_iter = 1;

					load_sm = LOAD_HEADER_MFM;
				}
				break;

			case LOAD_HEADER_MFM:
				load->header[0] = mfm_encode(load->header[0], true);
				load->header[1] = mfm_encode(load->header[1], false) & 0b1111111111011111;
				load->header[2] = mfm_encode(load->header[2], false);
				load->header[3] = mfm_encode(load->header[3], false);
				load->header[4] = mfm_encode(load->header[4], false);
				load->header[5] = mfm_encode(load->header[5], false);
				load->header[6] = mfm_encode(load->header[6], false);
				load->header[7] = mfm_encode(load->header[7], false);
				load->header[8] = mfm_encode(load->header[8], false);
				load->header[9] = mfm_encode(load->header[9], false);
				load_iter = 0;
				load_sm = LOAD_DATA_MFM;
				break;

			case LOAD_DATA_MFM:

				if ((load_iter >= 3) && (load_iter < 515)) {
					load->data[load_iter] = mfm_encode(cyl_data.track[cs].sector[next_sector].data[load_iter - 3]);
				} else {
					load->data[load_iter] = mfm_encode(load->data[load_iter]);
				}

				load_iter++;

				if ((load_iter >= 3) && (load_iter < 515)) {
					load->data[load_iter] = mfm_encode(cyl_data.track[cs].sector[next_sector].data[load_iter - 3]);
				} else {
					load->data[load_iter] = mfm_encode(load->data[load_iter]);
				}

				load_iter++;

				if ((load_iter >= 3) && (load_iter < 515)) {
					load->data[load_iter] = mfm_encode(cyl_data.track[cs].sector[next_sector].data[load_iter - 3]);
				} else {
					load->data[load_iter] = mfm_encode(load->data[load_iter]);
				}

				load_iter++;

				if ((load_iter >= 3) && (load_iter < 515)) {
					load->data[load_iter] = mfm_encode(cyl_data.track[cs].sector[next_sector].data[load_iter - 3]);
				} else {
					load->data[load_iter] = mfm_encode(load->data[load_iter]);
				}

				load_iter++;

				if ((load_iter >= 3) && (load_iter < 515)) {
					load->data[load_iter] = mfm_encode(cyl_data.track[cs].sector[next_sector].data[load_iter - 3]);
				} else {
					load->data[load_iter] = mfm_encode(load->data[load_iter]);
				}

				load_iter++;

				if (load_iter >= 520) {
					load->data[1] &= 0b1111111111011111;
					//digitalWrite(INDEX, LOW);
					//digitalWrite(INDEX, HIGH);
					load_sm = LOAD_IDLE;
				}
				break;

			case LOAD_SYNC:
				load->header[1] &= 0b1111111111011111;
				load->data[1] &= 0b1111111111011111;
				load_sm = LOAD_IDLE;
				break;
		}
	}
}

void create_track_store() {
	if (cyl_data.track) {
		for (int i = 0; i < cyl_data.heads; i++) {
			if (cyl_data.track[i].sector) {
				free(cyl_data.track[i].sector);
			}
		}
		free(cyl_data.track);
	}

	cyl_data.track = (struct track *)malloc(sizeof(struct track) * format->heads);
	for (int i = 0; i < format->heads; i++) {
		cyl_data.track[i].sector = (struct raw_sector *)malloc(sizeof(struct raw_sector) * format->sectors);
	}
}

void calculate_sector_crc(int head, int sector, int sectorsize) {
	uint32_t crc = 0xFFFFFFFF;
	uint32_t poly = format->data_poly;

	crc = crc32(0xA1, crc, poly);
	crc = crc32(0xFB, crc, poly);
	uint8_t *data = cyl_data.track[head].sector[sector].data;
	for (int i = 0; i < (0x80 << sectorsize); i++) {
		crc = crc32(data[i], crc, poly);
	}

	cyl_data.track[head].sector[sector].data_crc = crc;
}

void load_cyl(FsFile file, uint32_t cyl, uint32_t heads, uint32_t sectors, uint32_t sectorsize) {

	uint32_t ts = micros();

	uint32_t offset = cyl * heads * (0x80 << sectorsize);
	file.seekSet(offset);

	uint32_t bufsz = heads * sectors * (0x80 << sectorsize);

	Serial.printf("Loading %d bytes\n", bufsz);

    file.read(track_buffer, bufsz);

	uint32_t rts = micros() - ts;

	for (int head = 0; head < format->heads; head++) {
		for (int sector = 0; sector < format->sectors; sector++) {
			//calculate_sector_crc(head, sector, sectorsize);
		}
	}

	uint32_t cts = micros() - ts;

	Serial.printf("T: %d R: %u c: %u %f tps\r\n", cyl, rts, cts - rts, 1 / (cts / 1000000.0));

	
}

char *trim(char *str)
{
  char *end;

  // Trim leading space
  while(isspace((unsigned char)*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace((unsigned char)*end)) end--;

  // Write new null terminator character
  end[1] = '\0';

  return str;
}

int execute_file(const char *filename, CLIClient *client) {
	FsFile file;

	if (!open_sd_if_needed()) {
		errno = ENOENT;
		return 10;
	}

	if (!sd.exists(filename)) {
		errno = ENOENT;
		return 10;
	}

	file.open(filename);
	char line[1024];
	while (file.fgets(line, 1024)) {
		char *tline = trim(line);
		if (strlen(tline) == 0) continue;
		if (tline[0] == '#') continue;
		client->print(filename);
		client->print(": ");
		client->println(tline);
		client->parseCommand(tline);
	}
	file.close();
	return 0;
}

CLI_COMMAND(cli_source) {
	if (argc != 2) {
		dev->println("Usage: source <filename>");
		return 10;
	}

	if (execute_file(argv[1], dev) == 0) {
		return 0;
	}

	dev->println("File not found");
	return 10;
}


CLI_COMMAND(cli_status) {
	uint32_t sector_bytes = 8 + format->slen + 6 + format->header_postgap + format->data_postgap + 4;
	uint32_t total_clocks = ((sector_bytes * format->sectors) + format->track_pregap + format->track_postgap + 1) * 8;

	if ((argc == 2) && (strcmp(argv[1], "ini") == 0)) {
		dev->print("cyls="); dev->println(format->cyls);
		dev->print("heads="); dev->println(format->heads);
		dev->print("sectors="); dev->println(format->sectors);
		dev->print("sector_size="); dev->println(0x80 << format->sector_size);
		dev->print("track_pregap="); dev->println(format->track_pregap);
		dev->print("track_postgap="); dev->println(format->track_postgap);
		dev->print("header_postgap="); dev->println(format->header_postgap);
		dev->print("data_postgap="); dev->println(format->data_postgap);
		dev->print("data_rate="); dev->println(format->data_rate);
		dev->print("header_crc="); dev->println((format->flags & OPT_HEADER_CRC16) ? "16" : (format->flags & OPT_HEADER_CRC32) ? "32" : "ERROR");
		dev->print("data_crc="); dev->println((format->flags & OPT_DATA_CRC16) ? "16" : (format->flags & OPT_DATA_CRC32) ? "32" : "ERROR");
		dev->print("header_poly="); dev->println(format->header_poly, HEX);
		dev->print("data_poly="); dev->println(format->data_poly, HEX);
		return 0;
	}

	dev->print("Mounted image:          ");
	if (mounted_file) {
		mounted_file.printName(dev);
		dev->println();
	} else {
		dev->println("none");
	}

	float rpm = (format->data_rate / total_clocks) * 60.0;

	dev->print("Cylinders:              ");
	dev->println(format->cyls);
	dev->print("Heads:                  ");
	dev->println(format->heads);
	dev->print("Sectors:                ");
	dev->println(format->sectors);
	dev->print("Sector Size:            ");
	dev->print(0x80 << format->sector_size);
	dev->println(" bytes");

	dev->println();

	dev->print("Header CRC Bits:        "); 
	dev->println((format->flags & OPT_HEADER_CRC16) ? "16" : (format->flags & OPT_HEADER_CRC32) ? "32" : "ERROR");
	dev->print("Data CRC Bits:          ");
	dev->println((format->flags & OPT_DATA_CRC16) ? "16" : (format->flags & OPT_DATA_CRC32) ? "32" : "ERROR");
	dev->print("Header CRC Polynomial:  ");
	dev->println(format->header_poly, HEX);
	dev->print("Data CRC Polynomial:    ");
	dev->println(format->data_poly, HEX);

	dev->println();
	dev->print("Track Pregap:           ");
	dev->print(format->track_pregap);
	dev->println(" uS");

	dev->print("Track Postgap:          ");
	dev->print(format->track_postgap);
	dev->println(" uS");

	dev->print("Header Postgap:         ");
	dev->print(format->header_postgap);
	dev->println(" uS");

	dev->print("Data Postgap:           ");
	dev->print(format->data_postgap);
	dev->println(" uS");

	dev->println();

	dev->print("Actual RPM:             ");
	float p = format->idx_period / 1000000.0;
	float f = 1.0 / p;
	float r = f * 60;
	dev->println(r);

	dev->print("Requested Data Rate:    ");
	dev->print(format->data_rate);
	dev->println("Hz");
	dev->print("Actual Data Rate:       ");
	dev->print(F_CPU / format->clock_div / 20.0);
	dev->println("Hz");

	dev->print("Clock divider:          ");
	dev->println(format->clock_div);

	dev->println();
	dev->print("PIO:                    ");
	dev->println(PIO_NUM(pio));
	dev->print("State Machine:          ");
	dev->println(sm);
	dev->print("Offset:                 ");
	dev->println(offset);

	dev->println();
	dev->print("Load time:              ");
	dev->print(loadtime);
	dev->println("uS");

	dev->println();
	dev->print("Current C/H/S:          ");
	dev->print(current_cyl);
	dev->print("/");
	dev->print(current_head);
	dev->print("/");
	dev->println(current_sector);

	dev->print("MFM State Machine Run:  ");
	dev->println(run_mfm_sm ? "Yes" : "No");

	dev->println();
	dev->print("Known formats: ");
	for (struct format_list *scan = formats; scan; scan = scan->next) {
		dev->print(scan->fmt->name);
		dev->print(" ");
	}
	dev->println();

	return 0;
}

CLI_COMMAND(cli_set) {

	if (argc != 3) {
		dev->println("Usage: set <item> <value>");

		dev->println("Possible items:");
		dev->println("    track_pregap");
		dev->println("    track_posthap");
		dev->println("    header_postgap");
		dev->println("    data_postgap");
		dev->println("    data_rate");
		dev->println("    header_crc");
		dev->println("    data_crc");
		dev->println("    header_poly");
		dev->println("    data_poly");
		dev->println("    auto_return");
		return 10;
	}

	if (strcmp(argv[1], "track_pregap") == 0) {
		format->track_pregap = strtoul(argv[2], NULL, 10);
		return 0;
	}

	if (strcmp(argv[1], "track_postgap") == 0) {
		format->track_postgap = strtoul(argv[2], NULL, 10);
		return 0;
	}

	if (strcmp(argv[1], "header_postgap") == 0) {
		format->header_postgap = strtoul(argv[2], NULL, 10);
		return 0;
	}

	if (strcmp(argv[1], "data_postgap") == 0) {
		format->data_postgap = strtoul(argv[2], NULL, 10);
		return 0;
	}

	if (strcmp(argv[1], "data_rate") == 0) {
		float r = strtof(argv[2], NULL);
		float cd = F_CPU / r / 20.0;
		if (cd < 1) {
			dev->println("Data rate too high for the CPU clock");
			return 10;
		}
		format->data_rate = r;
		format->clock_div = cd;
		pio_sm_set_clkdiv(pio, sm, format->clock_div);
		return 0;
	}

	if (strcmp(argv[1], "header_crc") == 0) {
		int c = strtol(argv[2], NULL, 10);
		if (c == 16) {
			format->flags &= OPT_HEADER_CRC_MASK;
			format->flags |= OPT_HEADER_CRC16;
			return 0;
		}

		if (c == 32) {
			format->flags &= OPT_HEADER_CRC_MASK;
			format->flags |= OPT_HEADER_CRC32;
			return 0;
		}
		dev->println("Error: header_crc must be 16 or 32");
		return 10;
	}

	if (strcmp(argv[1], "data_crc") == 0) {
		int c = strtol(argv[2], NULL, 10);
		if (c == 16) {
			format->flags &= OPT_DATA_CRC_MASK;
			format->flags |= OPT_DATA_CRC16;
			return 0;
		}

		if (c == 32) {
			format->flags &= OPT_DATA_CRC_MASK;
			format->flags |= OPT_DATA_CRC32;
			return 0;
		}
		dev->println("Error: data_crc must be 16 or 32");
		return 10;
	}

	if (strcmp(argv[1], "header_poly") == 0) {
		format->header_poly = strtoul(argv[2], NULL, 16);
		return 0;
	}

	if (strcmp(argv[1], "data_poly") == 0) {
		format->data_poly = strtoul(argv[2], NULL, 16);
		return 0;
	}

	if (strcmp(argv[1], "auto_return") == 0) {
		format->auto_return = strncasecmp(argv[2], "Y", 1) == 0;
		return 0;
	}


	dev->println("Possible items:");
	dev->println("    track_pregap");
	dev->println("    track_posthap");
	dev->println("    header_postgap");
	dev->println("    data_postgap");
	return 10;
}

CLI_COMMAND(cli_ls) {
	if (!open_sd_if_needed()) {
		return 10;
	}
	sd.ls(LS_A | LS_DATE | LS_SIZE);
	return 0;
}

CLI_COMMAND(cli_create) {
	if (argc != 3) {
		dev->println("Usage: create <filename> <format>");
		dev->println("Create a new image file using the specified disk format");
		return 10;
	}
	if (!open_sd_if_needed()) {
		return 10;
	}


	struct disk_format *fmt = find_format_by_name(argv[2]);

	if (fmt == NULL) {
		dev->println("Unknown format");
		return 10;
	}

	if (sd.exists(argv[1])) {
		dev->println("File already exists");
		return 10;
	}

	uint32_t disk_size = 0x80 << fmt->sector_size;
	disk_size *= fmt->sectors;
	disk_size *= fmt->heads;
	disk_size *= fmt->cyls;

	dev->println("Creating new file, please wait...");
	FsFile newfile;
	newfile.open(argv[1], O_RDWR | O_CREAT);
	newfile.preAllocate(disk_size);
	newfile.close();
	return 0;
}

CLI_COMMAND(cli_mount) {
	if (argc != 3) {
		dev->println("Usage: mount <filename> <format>");
		dev->println("Mount an image file as the virtual disk");
		return 10;
	}
	if (!open_sd_if_needed()) {
		return 10;
	}

	if (!sd.exists(argv[1])) {
		dev->println("File not found");
		return 10;
	}

	run_mfm_sm = false;
	if (!mutex_enter_timeout_ms(&mfm_sm_running, 1000)) {
		dev->println("Timeout waiting for disk idle");
		return 10;
	}

	//while (mfm_sm_running);

	if (mounted_file) {
		mounted_file.close();
	}

	format = find_format_by_name(argv[2]);
	if (format == NULL) {
		dev->println("Format not known");
		return 10;
	}

	create_track_store();

	mounted_file = sd.open(argv[1], O_RDWR);
	current_cyl = 0;
	current_head = 0;

	uint32_t bps = 0x80 << format->sector_size;

	uint32_t bufsz = format->heads * format->sectors * bps;
	dev->printf("Track buffer size: %d\n", bufsz);

	track_buffer = (uint8_t *)malloc(bufsz);

    for (int head = 0; head < format->heads; head++) {
		int hoff = head * format->sectors;
        for (int sector = 0; sector < format->sectors; sector++) {
			cyl_data.track[head].sector[sector].data = track_buffer + ((hoff + sector) * bps);
        }
    }

	CRC32_init(format->data_poly);

        dev->print("cyls="); dev->println(format->cyls);
        dev->print("heads="); dev->println(format->heads);
        dev->print("sectors="); dev->println(format->sectors);
        dev->print("sector_size="); dev->println(0x80 << format->sector_size);
        dev->print("track_pregap="); dev->println(format->track_pregap);
        dev->print("track_postgap="); dev->println(format->track_postgap);
        dev->print("header_postgap="); dev->println(format->header_postgap);
        dev->print("data_postgap="); dev->println(format->data_postgap);
        dev->print("data_rate="); dev->println(format->data_rate);
        dev->print("header_crc="); dev->println((format->flags & OPT_HEADER_CRC16) ? "16" : (format->flags & OPT_HEADER_CRC32) ? "32" : "ERROR");
        dev->print("data_crc="); dev->println((format->flags & OPT_DATA_CRC16) ? "16" : (format->flags & OPT_DATA_CRC32) ? "32" : "ERROR");
        dev->print("header_poly="); dev->println(format->header_poly, HEX);
        dev->print("data_poly="); dev->println(format->data_poly, HEX);

	load_cyl(mounted_file, current_cyl, format->heads, format->sectors, format->sector_size);

	mutex_exit(&mfm_sm_running);
	run_mfm_sm = true;
	return 0;
}


void do_step() {
	if (format == NULL) return;
	//if(!mfm_sm_running) return;

	digitalWrite(SEEK_DONE, LOW);
	int dir = digitalRead(DIR);

	if (dir == 1 && current_cyl == 0) {
		// No can do.
		digitalWrite(SEEK_DONE, HIGH);
		return;
	}

	if (dir == 1) {
		target_cyl --;
	} else {
		target_cyl ++;
	}
	if (target_cyl >= format->cyls) {
		if (format->auto_return) {
			target_cyl = 0;
		} else {
			target_cyl = format->cyls - 1;
		}
	}
}


struct disk_format *find_format_by_name(const char *filename) {
	for (struct format_list *scan = formats; scan; scan = scan->next) {
		if (strcasecmp(filename, scan->fmt->name) == 0) {
			return scan->fmt;
		}
	}
	return NULL;
}

void append_format(struct disk_format *fmt) {
	if (formats == NULL) {
		formats = (struct format_list *)malloc(sizeof(struct format_list));
		formats->fmt = fmt;
		formats->next = NULL;
		return;
	}

	struct format_list *nf = (struct format_list *)malloc(sizeof(struct format_list));
	nf->fmt = fmt;
	nf->next = NULL;

	for (struct format_list *scan = formats; scan; scan = scan->next) {
		if (scan->next == NULL) {
			scan->next = nf;
			return;
		}
	}
}

struct disk_format *load_format(const char *filename) {
	FsFile file;

	file.open(filename);
	if (!file) {
		return NULL; 
	}

	struct disk_format *fmt = (struct disk_format *)malloc(sizeof(struct disk_format));
	memset(fmt, 0, sizeof(struct disk_format));

	char l[1024];
	while (file.fgets(l, 1024)) {
		char *tl = trim(l);

		if (strlen(tl) == 0) continue;
		if (tl[0] == '#') continue;
		if (tl[0] == ';') continue;
		if (tl[0] == '!') continue;

		char *key = strtok(tl, "=");
		char *val = strtok(NULL, "=");

		key = trim(key);
		val = trim(val);

		if (strcasecmp(key, "name") == 0) {
			fmt->name = strdup(val);
			continue;
		}

		if (strcasecmp(key, "comment") == 0) {
			fmt->comment = strdup(val);
			continue;
		}

		if (strcasecmp(key, "cyls") == 0) {
			fmt->cyls = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "heads") == 0) {
			fmt->heads = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "sectors") == 0) {
			fmt->sectors = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "sector_size") == 0) {
			int ss = strtoul(val, NULL, 0);
			switch (ss) {
				case 128: fmt->sector_size = SECTOR_128; break;
				case 256: fmt->sector_size = SECTOR_256; break;
				case 512: fmt->sector_size = SECTOR_512; break;
				case 1024: fmt->sector_size = SECTOR_1024; break;
			}
			continue;
		}

		if (strcasecmp(key, "index_width") == 0) {
			fmt->index_width = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "track_pregap") == 0) {
			fmt->track_pregap = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "track_postgap") == 0) {
			fmt->track_postgap = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "header_postgap") == 0) {
			fmt->header_postgap = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "data_postgap") == 0) {
			fmt->data_postgap = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "data_rate") == 0) {
			fmt->data_rate = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "header_crc") == 0) {
			int ss = strtoul(val, NULL, 0);
			switch (ss) {
				case 16: fmt->flags |= OPT_HEADER_CRC16; break;
				case 32: fmt->flags |= OPT_HEADER_CRC32; break;
			}
			continue;
		}

		if (strcasecmp(key, "data_crc") == 0) {
			int ss = strtoul(val, NULL, 0);
			switch (ss) {
				case 16: fmt->flags |= OPT_DATA_CRC16; break;
				case 32: fmt->flags |= OPT_DATA_CRC32; break;
			}
			continue;
		}

		if (strcasecmp(key, "header_poly") == 0) {
			fmt->header_poly = strtoul(val, NULL, 0);
			continue;
		}

		if (strcasecmp(key, "data_poly") == 0) {
			fmt->data_poly = strtoul(val, NULL, 0);
			continue;
		}
	}

	file.close();
	return fmt;
}

void load_formats(CLIClient *dev) {
	char name[1024];
	FsFile f;
	FsFile dir;
	if (open_sd_if_needed()) {
		dir.open("/", O_RDONLY);
		dir.rewind();
		while (f.openNext(&dir, O_RDONLY)) {
			f.getName(name, 1024);
			f.close();
			int s = strlen(name);
			if (s > 4) {
				if ((name[s - 4] == '.') &&
					(name[s - 3] == 'f') &&
					(name[s - 2] == 'm') &&
					(name[s - 1] == 't')) {
						dev->println(name);
						struct disk_format *fmt = load_format(name);
						if (fmt == NULL) {
							dev->println("Error loading format");
						} else {
							append_format(fmt);
						}
				}
			}
		}
		dir.close();
	}
}

void factory_formats() {
	struct disk_format *rd54 = (struct disk_format *)malloc(sizeof(struct disk_format));


	rd54->name = strdup("RD54");
	rd54->comment = strdup("DEC RD54 Hard Drive");
	rd54->cyls=1225;
	rd54->heads=15;
	rd54->sectors=17;
	rd54->sector_size=SECTOR_512;
	rd54->index_width=10;
	rd54->track_pregap=290;
	rd54->track_postgap=1100;
	rd54->header_postgap=41;
	rd54->data_postgap=911;
	rd54->data_rate=5000000;
	rd54->flags = OPT_HEADER_CRC16 | OPT_DATA_CRC32;
	rd54->header_poly=0x1021;
	rd54->data_poly=0xa00805;
	rd54->auto_return = true;

	append_format(rd54);
}

CLI_COMMAND(cli_eject) {
	if (mounted_file) {
		mounted_file.close();
		if (track_buffer) {
			free(track_buffer);
			track_buffer = NULL;
		}
	}
	sd.end();
	_sd_is_open = false;
	dev->println("It is now safe to remove the SD card.");
	return 0;
}

CLI_COMMAND(cli_load_formats) {
	load_formats(dev);
	return 0;
}

CLI_COMMAND(cli_seek) {
	if (argc != 2) {
		dev->println("Usage: seek <track>");
		return 10;
	}

	int track = strtol(argv[1], NULL, 0);

	if ((track < 0) || (track >= format->cyls)) {
		dev->println("Track out of range");
		return 10;
	}

	target_cyl = track;

	return 0;
}

void setup() {
	mutex_init(&mfm_sm_running);
    datastreamPgm.prepare(&pio, &sm, &offset);
    datastream_program_init(pio, sm, offset, DOUT);
    pio_sm_set_enabled(pio, sm, true);

	pio->txf[sm] = mfm_encode(0x00);

	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	Serial.begin(115200);

	//format = &RD54;
	factory_formats();
	format = formats->fmt;
	load_formats(console);
	create_track_store();



	format->slen = 0x80 << format->sector_size;
	format->tlen = format->slen * format->sectors;
	format->clock_div = F_CPU / format->data_rate / 20.0;
	pio_sm_set_clkdiv(pio, sm, format->clock_div);

    multicore_launch_core1(second_cpu_thread);

	CLI.setDefaultPrompt("RTmFM> ");
	console = CLI.addClient(Serial);

	CLI.addCommand("status", cli_status);
	CLI.addCommand("set", cli_set);
	CLI.addCommand("ls", cli_ls);
	CLI.addCommand("dir", cli_ls);
	CLI.addCommand("create", cli_create);
	CLI.addCommand("mount", cli_mount);
	CLI.addCommand("source", cli_source);
	CLI.addCommand("eject", cli_eject);
	CLI.addCommand("lf", cli_load_formats);
	CLI.addCommand("seek", cli_seek);

	pinMode(STEP, INPUT);
	pinMode(DIR, INPUT);
	pinMode(SEEK_DONE, OUTPUT);
	digitalWrite(SEEK_DONE, HIGH);
	pinMode(TRACK0, OUTPUT);
	digitalWrite(TRACK0, current_cyl == 0);

	attachInterrupt(STEP, do_step, FALLING);

	if (open_sd_if_needed()) {
		if (sd.exists("autoexec.bat")) {
			execute_file("autoexec.bat", console);
		}
	}
}

void loop() {

	CLI.process();

		cli();
		uint32_t tmp_target = target_cyl;
		sei();

		if (tmp_target != current_cyl) {
			while (tmp_target != current_cyl) {
				current_cyl = tmp_target;
				run_mfm_sm = false;
				if (!mutex_enter_timeout_ms(&mfm_sm_running, 1000)) {
					Serial.println("Timeout waiting for disk idle");
				} else {
					load_cyl(mounted_file, current_cyl, format->heads, format->sectors, format->sector_size);
					digitalWrite(TRACK0, current_cyl == 0);
					cli();
					uint32_t tmp_target = target_cyl;
					sei();
					mutex_exit(&mfm_sm_running);
					run_mfm_sm = true;
				}
			}
			digitalWrite(SEEK_DONE, HIGH);
		}
}
