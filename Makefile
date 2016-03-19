# MCU Selection
#MCU = atmega32u2
#MCU = atmega168a
MCU = atmega164p

# Clock Speed
CLOCK = 8000000UL

## Objects that must be built in order to link

CC = avr-gcc
CPP = avr-g++


## Compile options common for all C compilation units.
CFLAGS = -Wall -gdwarf-2 -Os
#CFLAGS = -g	#Debug
#CFLAGS += -Os	# Optimize size
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d # Dependancy files
CFLAGS += -Iukhasnet-rfm69

## Linker flags
LDFLAGS = 

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings




## Build
.PHONY: all
all:
	@echo "Use make PROJECT where project is one of:"
	@echo "  door-led"
	@echo "  gps-isr-test"
	@echo "  rfm-test"
	@echo "These will build and program the image"
	@echo "Add .hex to build the image to flash"

spi_conf.c:
	$(error You need to link a suitable spi_conf.c file from ukhasnet-rfm69/spi_conf/)

ukhasnet-rfm69/ukhasnet-rfm69.c:
	$(error You need to checkout the ukhasnet-rfm69 submodule)

## Compile
ukhasnet-rfm69.o: ukhasnet-rfm69/ukhasnet-rfm69.c
	$(CC) -mmcu=$(MCU) -DF_CPU=$(CLOCK) $(CFLAGS) -c  $<

%.o: %.c
	$(CC) -mmcu=$(MCU) -DF_CPU=$(CLOCK) $(CFLAGS) -c  $<

.PHONY: door-led
door-led: door-led.elf door-led.hex door-led.eep
	sudo avrdude -c avrispmkii -p m164p -P usb -U flash:w:$@.hex

.PHONY: gps-isr-test
gps-isr-test: gps-isr-test.elf gps-isr-test.hex gps-isr-test.eep
	sudo avrdude -c avrispmkii -p m164p -P usb -U flash:w:$@.hex

.PHONY: rfm-test
rfm-test: rfm-test.elf rfm-test.hex rfm-test.eep
	sudo avrdude -c avrispmkii -p m164p -P usb -U flash:w:$@.hex


##Link
door-led.elf: door-led.o
	 $(CC) -mmcu=$(MCU)  $(LDFLAGS) $^ -o $@
gps-isr-test.elf: gps-isr-test.o
	 $(CC) -mmcu=$(MCU)  $(LDFLAGS) $^ -o $@
rfm-test.elf: rfm-test.o ukhasnet-rfm69.o spi_conf.o
	 $(CC) -mmcu=$(MCU)  $(LDFLAGS) $^ -o $@


%.hex: %.elf
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: %.elf
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: %.elf
	avr-objdump -h -S $< > $@

#size: ${TARGET}
	#avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
#$(shell rm {ukhasnet-rf69,rfm-test}.{eep,elf,o,hex} dep/*)
.PHONY: clean
clean:
	-rm *.o
	-rm *.elf
	

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

