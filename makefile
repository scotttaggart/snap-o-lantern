#NOTE: This file changes the fuse values of the chip to enable brown-out detection.
#fuse settings are hard-coded into the bottom lines; change them only with care.

PRG            = snap
OBJ            = snap.o
MCU_TARGET     = attiny2313
AVRDUDE_TARGET = t2313 

PORT		   = usb			# Will need to be adjusted for your programmer!
PROGRAMMER     = usbasp		# Will need to be adjusted for your programmer!

OPTIMIZE       = -Os

DEFS           =
LIBS           =


# You should not have to change anything below here.

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text #eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak *.hex *.bin *.srec
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec


ehex:  $(PRG)_eeprom.hex
#ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

#%_eeprom.srec: %.elf
#	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@


# command to program chip (invoked by running "make program")
program: 
	# for fuses values, see: http://www.engbedded.com/fusecalc/
	# 			normal: -U lfuse:w:0xE4:m  -U hfuse:w:0xDF:m -U efuse:w:0xff:m \
	#  clock out on D2: -U lfuse:w:0xA4:m  -U hfuse:w:0xDF:m -U efuse:w:0xff:m \
	avrdude -p $(AVRDUDE_TARGET) -c $(PROGRAMMER) -P $(PORT) -v -e -b 115200  \
	 -U lfuse:w:0xE4:m  -U hfuse:w:0xDF:m -U efuse:w:0xff:m \
	 -U flash:w:$(PRG).hex	