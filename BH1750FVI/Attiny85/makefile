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
libbh1750fvi.lss \

SIZEDUMMY += \
sizedummy \


# All Target
all: libbh1750fvi.a secondary-outputs

dependents:
	-cd /home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85 && $(MAKE) all
	-cd /home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85 && $(MAKE) all

# Tool invocations
libbh1750fvi.a: $(OBJS) /home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85/libattinycore.a /home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85/libtinywirem.a
	@echo 'Building target: $@'
	@echo 'Invoking: AVR Archiver'
	avr-ar -r  "libbh1750fvi.a" $(OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

libbh1750fvi.lss: libbh1750fvi.a
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S libbh1750fvi.a  >"libbh1750fvi.lss"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: libbh1750fvi.a
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny85 libbh1750fvi.a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(ARCHIVES)$(S_DEPS)$(LSS)$(C_DEPS)$(CC_DEPS)$(C++_DEPS)$(OBJS)$(CXX_DEPS)$(ASM_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(CPP_DEPS) libbh1750fvi.a
	-@echo ' '

secondary-outputs: $(LSS) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
/home/gionata/workspace_Arduino/Core/AttinyCore/Attiny85/libattinycore.a:
/home/gionata/workspace_Arduino/Utility/TinyWireM/Attiny85/libtinywirem.a:

-include ../makefile.targets
