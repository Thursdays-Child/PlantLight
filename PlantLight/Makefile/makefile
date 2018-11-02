CXX=avr-g++

CORE_DIR=../../../../Core/ATTinyCore/
PROJECT_DIR=..
TINYWIRE_DIR=../../../../Libraries/I2C/TinyWireM/
DS1307RTC_DIR=../../../../Libraries/Sensors/BH1750FVI/
BH1750FVI_DIR=../../../../Libraries/Time/DS1307RTC/
LIB_DIR=Attiny85

# avr tools path
AVR_OBJCOPY=/usr/bin/avr-objcopy
AVR_SIZE=/usr/bin/avr-size
AVRDUDE=/usr/bin/avrdude

# Debug flag
#DEBUG=-DDEBUG

# CPU type and speed
MCU=attiny85
CPU_SPEED=1000000UL


# Include (dependencies: Attiny core, DS1307 RTC clock , BH1750FVI light sensor, TINYWIRE I2C bus)
INCLUDE=-I$(CORE_DIR) -I$(DS1307RTC_DIR) -I$(TINYWIRE_DIR) -I$(BH1750FVI_DIR)

# Libraries (dependencies: Attiny core, DS1307 RTC clock , BH1750FVI light sensor, TINYWIRE I2C bus)
LIBS=-L$(CORE_DIR)/ATTiny85 -L$(TINYWIRE_DIR)/$(LIB_DIR)\
-L$(DS1307RTC_DIR)/$(LIB_DIR) -L$(BH1750FVI_DIR)/$(LIB_DIR)\
-lbh1750fvi -lds1307rtc -ltinywirem -lattinycore

# Source file and application name
OBJ=main
TARGET=PlantLight

CFLAGS=-Wall -Wno-unused-local-typedefs -Os -fpack-struct -fshort-enums\
-funsigned-char -funsigned-bitfields -fno-exceptions -ffunction-sections\
-fdata-sections -fno-use-cxa-atexit -mmcu=$(MCU) -DF_CPU=$(CPU_SPEED)\
-MMD -MP -MF"$(OBJ).d" -MT"$(OBJ).d" $(DEBUG)

CLINKFLAGS=-Wl,-Map,$(TARGET).map,--cref -Wl,-gc-sections -mmcu=$(MCU)


default: build sizedummy upload 

build: $(TARGET).hex
	
$(TARGET).hex: $(TARGET).elf
	@echo 'Create Flash image (ihex format)'
	$(AVR_OBJCOPY) -R .eeprom -O ihex $< $@
	@echo 'Finished building target: $@'
	@echo ' '

$(TARGET).elf: $(OBJ).o
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	$(CXX) $(CLINKFLAGS) $(INCLUDE) $^ -o $@ $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	
%.o: ../%.cpp
	$(CXX) $< $(CFLAGS) $(INCLUDE) -c -o $@
	
upload:
	@echo 'Invoking: AVRDude'
	$(AVRDUDE) -p$(MCU) -cusbasp -Uflash:w:$(TARGET).hex:a -Ueeprom:w:../eeprom_CET_CEST_rules.hex:a
	@echo 'Finished building: $@'
	@echo ' '
	
sizedummy: $(TARGET).elf
	@echo 'Invoking: Print Size'
	$(AVR_SIZE) --format=avr --mcu=$(MCU) $(TARGET).elf
	@echo 'Finished building: $@'
	@echo ' '	
	
clean:
	@echo -n Cleaning ...
	$(shell rm $(TARGET).elf 2> /dev/null)
	$(shell rm $(TARGET).hex 2> /dev/null)
	$(shell rm $(TARGET).map 2> /dev/null)
	$(shell rm *.o 2> /dev/null)
	$(shell rm *.d 2> /dev/null)
	@echo " done"
	
