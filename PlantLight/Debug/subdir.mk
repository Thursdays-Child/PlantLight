################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../main.cpp 

OBJS += \
./main.o 

CPP_DEPS += \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"/home/gionata/workspace_Arduino/Plant_Light/PlantLight" -I"/home/gionata/workspace_Arduino/Core/AttinyCore" -I"/home/gionata/workspace_Arduino/Utility/TinyWireM" -I"/home/gionata/workspace_Arduino/Plant_Light/BH1750FVI" -I"/home/gionata/workspace_Arduino/Plant_Light/DS1307RTC" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fno-exceptions -ffunction-sections -fdata-sections -fno-use-cxa-atexit -Wno-unused-local-typedefs -mmcu=attiny85 -DF_CPU=1000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


