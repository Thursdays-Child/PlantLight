################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../BH1750FVI.cpp 

OBJS += \
./BH1750FVI.o 

CPP_DEPS += \
./BH1750FVI.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/gionata/workspace_Arduino/Plant_Light/BH1750FVI" -I"/home/gionata/workspace_Arduino/Core/AttinyCore" -I"/home/gionata/workspace_Arduino/Utility/TinyWireM" -DBH1750_DEBUG=1 -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -mmcu=attiny85 -DF_CPU=1000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


