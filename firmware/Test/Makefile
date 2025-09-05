BOARD=rp2040:rp2040:rpipico2
#OPTS=PSRAM=enabled,FlashSize=4M,PartitionScheme=min_spiffs,CPUFreq=240
SKETCH=Test
PORT?=/dev/ttyACM0

PIOASM=~/.arduino15/packages/rp2040/tools/pqt-pioasm/4.0.1-8ec9d6f/pioasm



SUBDIR=$(subst :,.,$(BOARD))
BIN=bin/$(SKETCH).ino.bin
#IP=$(shell avahi-resolve -n $(HOST).local | cut -f2 -d'	')

$(BIN): $(SKETCH).ino datastream.h
	#arduino-cli compile --fqbn $(BOARD) --board-options "freq=300,arch=riscv" --output-dir bin --build-path build
	arduino-cli compile --fqbn $(BOARD) --board-options "freq=300" --output-dir bin --build-path build

upload: $(BIN)
	arduino-cli upload --fqbn $(BOARD) -p $(PORT) --input-dir bin

clean:
	rm -rf build bin

watch:
	echo ${SKETCH}.ino | entr -c -s 'make'

datastream.h: datastream.pio
	${PIOASM} datastream.pio datastream.h
