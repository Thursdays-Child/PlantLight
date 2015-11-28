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
LSS += \
libds1307rtc.lss \

SIZEDUMMY += \
sizedummy \


# All Target
all: libds1307rtc.a secondary-outputs

dependents:
	-cd /home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85 && $(MAKE) all
	-cd /home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85 && $(MAKE) all

# Tool invocations
libds1307rtc.a: $(OBJS) /home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85/libattinycore.a /home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85/libtinywirem.a
	@echo 'Building target: $@'
	@echo 'Invoking: AVR Archiver'
	avr-ar -r  "libds1307rtc.a" $(OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

libds1307rtc.lss: libds1307rtc.a
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S libds1307rtc.a  >"libds1307rtc.lss"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: libds1307rtc.a
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny85 libds1307rtc.a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(ARCHIVES)$(S_DEPS)$(LSS)$(C_DEPS)$(CC_DEPS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) libds1307rtc.a
	-@echo ' '

secondary-outputs: $(LSS) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
/home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85/libattinycore.a:
/home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85/libtinywirem.a:

-include ../makefile.targets
