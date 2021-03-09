
FILE_NAME = esc

DEVICE = atmega328p
F_CPU = 16000000 #hz
COMPILE_FLAGS = -mmcu=$(DEVICE) -DF_CPU=$(F_CPU) -Wall #-fno-aggressive-loop-optimizations

L_FUSE = 0xE2
H_FUSE = 0xDF

AVRDUDE_FLAGS = -c usbtiny -p $(DEVICE)

all: $(FILE_NAME).hex

program: flash fuse

flash: $(FILE_NAME).hex
	avrdude $(AVRDUDE_FLAGS) -U flash:w:$(FILE_NAME).hex:i

fuse:
	avrdude $(AVRDUDE_FLAGS) -U hfuse:w:$(H_FUSE):m -U lfuse:w:$(L_FUSE):m

flashArduino:
	avrdude -c arduino -p $(DEVICE) -P /dev/ttyUSB0 -b 57600 -D -U flash:w:$(FILE_NAME).hex

$(FILE_NAME).hex: $(FILE_NAME).elf
	avr-objcopy -O ihex $(FILE_NAME).elf $(FILE_NAME).hex

$(FILE_NAME).elf: $(FILE_NAME).c
	avr-gcc -c $(FILE_NAME).c -Os $(COMPILE_FLAGS)
	avr-gcc -o $(FILE_NAME).elf $(FILE_NAME).o $(COMPILE_FLAGS)

clean:
	rm *.o *.elf *.hex
