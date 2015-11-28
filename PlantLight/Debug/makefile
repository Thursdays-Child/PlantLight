################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
FLASH_IMAGE += \
PlantLight.hex \

SIZEDUMMY += \
sizedummy \

AVRDUDEDUMMY += \
avrdudedummy \


# All Target
all: PlantLight.elf secondary-outputs

dependents:
	-cd /home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85 && $(MAKE) clean all
	-cd /home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85 && $(MAKE) clean all

# Tool invocations
PlantLight.elf: $(OBJS) $(USER_OBJS) /home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85/libattinycore.a /home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85/libtinywirem.a
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,PlantLight.map,--cref -mrelax -Wl,--gc-sections -Wl,-gc-sections -L"/home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85" -L"/home/gionata/workspace_Arduino/Plant_Light/BH1750FVI/Attiny85" -L"/home/gionata/workspace_Arduino/Plant_Light/DS1307RTC/Attiny85" -L"/home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85" -mmcu=attiny85 -o "PlantLight.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

PlantLight.hex: PlantLight.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex PlantLight.elf  "PlantLight.hex"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: PlantLight.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny85 PlantLight.elf
	@echo 'Finished building: $@'
	@echo ' '

avrdudedummy: PlantLight.elf
	@echo 'Invoking: AVRDude'
	/usr/bin/avrdude -pt85 -cusbasp -Uflash:w:PlantLight.hex:a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(AVRDUDEDUMMY)$(S_DEPS)$(C_DEPS)$(FLASH_IMAGE)$(CC_DEPS)$(ELFS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) PlantLight.elf
	-@echo ' '

secondary-outputs: $(FLASH_IMAGE) $(SIZEDUMMY) $(AVRDUDEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
/home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85/libattinycore.a:
/home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85/libtinywirem.a:

-include ../makefile.targets
