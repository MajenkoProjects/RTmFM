#include <CLI.h>

#include "datastream.h"

#define SECTOR_128 0
#define SECTOR_256 1
#define SECTOR_512 2
#define SECTOR_1024 3


static PIOProgram datastreamPgm(&datastream_program);

PIO pio;
int sm;
int offset;


#define POLY16 0x1021
#define POLY32 0xa00805

#define CQ16(V, C) C = crc16(V, C); queue(mfm_encode(V))
#define CQ32(V, C) C = crc32(V, C); queue(mfm_encode(V))

#define NUM_SECTORS 17

#define TRACK_SIZE (512 * NUM_SECTORS)

uint8_t *track_data;


struct disk_format {
	uint16_t cyls;
	uint8_t heads;
	uint8_t sectors;
	uint8_t sector_size;
	uint8_t track_pregap;
	uint8_t track_postgap;
	uint8_t header_postgap;
	uint8_t data_postgap;
	float data_rate;

	// Calculated data - not to be filled.

	uint32_t tlen;
	uint32_t slen;
	uint32_t idx_ts;
	uint32_t idx_period;
	float clock_div;
};


struct disk_format RD54 = {
	1225, 		// Cyl
	15, 		// Heads
	17, 		// Sectors,
	SECTOR_512, // Bytes per sector
	151, 		// Track pregap
	151, 		// Track postgap
	16, 		// Header postgap
	50,			// Data postgap
	5000000,	// Data Rate
};
	

struct disk_format *format;


void bindump(uint16_t v) {
	for (int i = 0; i < 16; i++) {
		v <<= 1;
	}
}

uint16_t crc16(uint8_t val, uint16_t crc)
{

	uint16_t xval = val;

   int j;
   crc = crc ^ (xval << 8);
   for (j = 1; j <= 8; j++) {   // Assuming 8 bits per val
      if (crc & 0x8000) {   // if leftmost (most significant) bit is set
         crc = (crc << 1) ^ POLY16;
      } else {
         crc = crc << 1;
      }
   }
   return crc;
}

uint32_t crc32(uint8_t val, uint32_t crc)
{

   int j;
   crc = crc ^ (val << 24);
   for (j = 1; j <= 8; j++) {   // Assuming 8 bits per val
      if (crc & 0x80000000) {   // if leftmost (most significant) bit is set
         crc = (crc << 1) ^ POLY32;
      } else {
         crc = crc << 1;
      }
   }
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

uint16_t mfm_encode(uint8_t b) {

	uint16_t out = 0;

	for (int i = 0; i < 8; i++) {
		out <<= 2;
		out |= mfm_encode_bit(b & 0x80);
		b <<= 1;
	}

	return out;
}

static inline void queue(uint16_t val) {
	while(pio_sm_is_tx_fifo_full(pio, sm));
	pio->txf[sm] = val;
}

void sync() {
	uint16_t sync = mfm_encode(0xA1);
	sync &= 0b1111111111011111;
	queue(sync);

}

void zero_pad() {
	queue(mfm_encode(0x00));
}

void send_header(uint16_t cyl, uint8_t head, uint8_t sector, uint8_t size) {
	uint16_t header_crc = 0xFFFF;

	zero_pad();

	sync();
	header_crc = crc16(0xA1, header_crc);

	CQ16(0xFE, header_crc);
	CQ16(cyl & 0xFF, header_crc);
	CQ16(((cyl >> 4) & 0xF0) | (head & 0x0F), header_crc);
	CQ16(sector, header_crc);
	CQ16(size, header_crc);

	queue(mfm_encode((header_crc >> 8) & 0xFF));
	queue(mfm_encode(header_crc & 0xFF));

	zero_pad();
}

void send_data(uint8_t *data, uint16_t len) {
	uint32_t data_crc = 0xFFFFFFFF;

	zero_pad();
	sync();
	data_crc = crc32(0xA1, data_crc);

	CQ32(0xFB, data_crc);

	for (int i = 0; i < len; i++) {
		CQ32(data[i], data_crc);
	}

	queue(mfm_encode((data_crc >> 24) & 0xFF));
	queue(mfm_encode((data_crc >> 16) & 0xFF));
	queue(mfm_encode((data_crc >> 8) & 0xFF));
	queue(mfm_encode(data_crc & 0xFF));

	zero_pad();

}

void send_sector(uint16_t cyl, uint8_t head, uint8_t sector, uint8_t size, uint8_t *data, uint8_t pad) {
	send_header(cyl, head, sector, size);

	for (int i = 0; i < pad; i++) {
		zero_pad();
	}

	uint16_t len = 0x80 << size;
	send_data(data, len);
}


void second_cpu_thread() {
	while (1) {
		for (int i = 0; i < format->track_postgap; i++) {
			zero_pad();
		}
		format->idx_period = micros() - format->idx_ts;
		format->idx_ts = micros();
		digitalWrite(13, LOW);
		zero_pad();
		digitalWrite(13, HIGH);
		for (int i = 0; i < format->track_pregap; i++) {
			zero_pad();
		}

		for (int sector = 0; sector < format->sectors; sector++) {

			send_sector(0, 0, sector, format->sector_size, track_data + (sector * format->slen), format->header_postgap);

			for (int i = 0; i < format->data_postgap; i++) {
				zero_pad();
			}
		}
	}
}


CLI_COMMAND(cli_status) {
	uint32_t sector_bytes = 8 + format->slen + 6 + format->header_postgap + format->data_postgap + 4;
	uint32_t total_clocks = ((sector_bytes * format->sectors) + format->track_pregap + format->track_postgap + 1) * 8;

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
	dev->print("Track Pregap:           ");
	dev->print(format->track_pregap);
	dev->println(" bytes");

	dev->print("Track Postgap:          ");
	dev->print(format->track_postgap);
	dev->println(" bytes");

	dev->print("Header Postgap:         ");
	dev->print(format->header_postgap);
	dev->println(" bytes");

	dev->print("Data Postgap:           ");
	dev->print(format->data_postgap);
	dev->println(" bytes");

	dev->println();

	dev->print("Total clocks per track: ");
	dev->println(total_clocks);

	dev->print("Calculated RPM:         ");
	dev->println(rpm);

	dev->print("Actual RPM:             ");
	float p = format->idx_period / 1000000.0;
	float f = 1.0 / p;
	float r = f * 60;
	dev->println(r);

	dev->print("Requested Data Rate:    ");
	dev->print(format->data_rate);
	dev->println("MHz");
	dev->print("Actual Data Rate:       ");
	dev->print(F_CPU / format->clock_div / 20.0);
	dev->println("MHz");

	dev->print("Clock divider:          ");
	dev->println(format->clock_div);

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


	dev->println("Possible items:");
	dev->println("    track_pregap");
	dev->println("    track_posthap");
	dev->println("    header_postgap");
	dev->println("    data_postgap");
	return 10;
}

void setup() {
    datastreamPgm.prepare(&pio, &sm, &offset);
    datastream_program_init(pio, sm, offset);
    pio_sm_set_enabled(pio, sm, true);

	pio->txf[sm] = mfm_encode(0x00);

	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	Serial.begin(115200);

	format = &RD54;

	format->slen = 0x80 << format->sector_size;
	format->tlen = format->slen * format->sectors;

	track_data = (uint8_t *)malloc(format->tlen);

	for (int i = 0; i < format->tlen; i++) {
		track_data[i] = rand();
	}


	format->clock_div = F_CPU / format->data_rate / 20.0;
	pio_sm_set_clkdiv(pio, sm, format->clock_div);

    multicore_launch_core1(second_cpu_thread);

	Serial.begin(115200);
	CLI.setDefaultPrompt("RTmFM> ");
	CLI.addClient(Serial);

	CLI.addCommand("status", cli_status);
	CLI.addCommand("set", cli_set);
}

void loop() {

	CLI.process();
}
