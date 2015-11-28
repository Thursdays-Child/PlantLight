################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../DS1307RTC.cpp 

OBJS += \
./DS1307RTC.o 

CPP_DEPS += \
./DS1307RTC.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/gionata/workspace_Arduino/Plant_Light/DS1307RTC" -I"/home/gionata/workspace_Arduino/Core/AttinyCore" -I"/home/gionata/workspace_Arduino/Utility/TinyWireM" -DBH1750_DEBUG=1 -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=attiny85 -DF_CPU=1000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


