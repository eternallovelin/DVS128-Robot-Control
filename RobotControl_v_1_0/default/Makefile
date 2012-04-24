###############################################################################
# Makefile for the project RobotControl_v_1_0
###############################################################################

## General Flags
PROJECT = RobotControl_v_1_0
MCU = atmega2561
TARGET = RobotControl_v_1_0.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                       -DF_CPU=16000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=RobotControl_v_1_0.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\Documents and Settings\Ivan konstantinov\My Documents\RobotControl_v_1_0\zigbee_src_winavr\include" 

## Library Directories
LIBDIRS = -L"C:\Documents and Settings\Ivan konstantinov\My Documents\RobotControl_v_1_0\zigbee_src_winavr\lib" 

## Libraries
LIBS = -ldynamixel -lzigbee 

## Objects that must be built in order to link
OBJECTS = robot_control_v_1_0.o dynamixel_control.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) RobotControl_v_1_0.hex RobotControl_v_1_0.eep RobotControl_v_1_0.lss size

## Compile
robot_control_v_1_0.o: ../robot_control_v_1_0.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

dynamixel_control.o: ../dynamixel_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) RobotControl_v_1_0.elf dep/* RobotControl_v_1_0.hex RobotControl_v_1_0.eep RobotControl_v_1_0.lss RobotControl_v_1_0.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

