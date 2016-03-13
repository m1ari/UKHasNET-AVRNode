## General Flags
PROJECT = gps-isr-test
#MCU = atmega32u2
#MCU = atmega168a
MCU = atmega164p
## Objects that must be built in order to link
OBJECTS = gps-isr-test.o

TARGET = $(PROJECT).elf


CC = avr-gcc
CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = -Wall -gdwarf-2 -Os
#CFLAGS = -g	#Debug
#CFLAGS += -Os	# Optimize size
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d # Dependancy files

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = 
LDFLAGS += 


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings



## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(PROJECT).elf $(PROJECT).hex $(PROJECT).eep 
# size

## Compile
%.o: %.c
	$(CC) -mmcu=$(MCU) $(CFLAGS) -c  $<

##Link
$(PROJECT).elf: $(OBJECTS)
	 $(CC) -mmcu=$(MCU)  $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) $(PROJECT).elf $(PROJECT).hex $(PROJECT).eep dep/*

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

