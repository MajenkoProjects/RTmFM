/*
 * PIO
 *
 *
 * Fifo format
 * 19 bits of address
 * 8 bits of current data value
 * 3 bits of head number
 * 2 bit of pin data
 *
 * Shift 1 bit from OSR into pins
 * Shift 11 bits from OSR into ISR
 * Shift 3 bits from pins into ISR
 *
 * PULL block
 * OUT pins 1
 * OUT ISR 11
 * IN pin 3
 * PUSH noblock
 *
 * Software:
 * Write to fifo data, head and current bit
 * Read from fifo.
 * Test write bit
 * If set, set or clear affected bit
 * Shift data to 8 bits and replace in buffer
 */

#include "datastream.h"

static PIOProgram datastreamPgm(&datastream_program);

PIO pio;
int sm;
int offset;

uint16_t header_crc;

#define POLY16 0x1021
#define POLY32 0xa00805

static inline void w() {
	while(pio_sm_is_tx_fifo_full(pio, sm));
}

void bindump(uint16_t v) {
	for (int i = 0; i < 16; i++) {
		Serial.print((v & 0x8000) ? "1" : "0");
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

	//Serial.print(">>> ");
	//Serial.println(b, HEX);

	for (int i = 0; i < 8; i++) {
		out <<= 2;
		//Serial.println(b, BIN);
		out |= mfm_encode_bit(b & 0x80);
		b <<= 1;
	}

	//Serial.print("Last bit: ");
	//Serial.println(last_bit);

	//Serial.print("Output: ");
	//Serial.println(out, BIN);

	return out;
}

void csend(uint8_t v) {
	w();
	pio->txf[sm] = mfm_encode(v);
	header_crc = crc16(v, header_crc);
}

void setup() {
    datastreamPgm.prepare(&pio, &sm, &offset);
    datastream_program_init(pio, sm, offset);
    pio_sm_set_enabled(pio, sm, true);

	pio->txf[sm] = mfm_encode(0x00);

	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	Serial.begin(115200);
}

void loop() {
	delay(1);

	digitalWrite(13, HIGH);
	digitalWrite(13, LOW);

	last_bit = 0;
	w();
	pio->txf[sm] = mfm_encode(0x00);

	header_crc = 0xFFFF;
	header_crc = crc16(0xA1, header_crc);
	uint16_t sync = mfm_encode(0xA1);
	//sync &= 0b1111101111111111;
	sync &= 0b1111111111011111;

	pio->txf[sm] = sync;
	csend(0xFE);
	csend(0x00);
	csend(0x00);
	csend(0x03);
	csend(0x02);
	w();
	pio->txf[sm] = mfm_encode(header_crc >> 8);
	w();
	pio->txf[sm] = mfm_encode(header_crc & 0xff);
	w();
	pio->txf[sm] = mfm_encode(0x00);

}
