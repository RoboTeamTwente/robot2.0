################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/PuttyInterface/PuttyInterface.c 

OBJS += \
./Src/PuttyInterface/PuttyInterface.o 

C_DEPS += \
./Src/PuttyInterface/PuttyInterface.d 


# Each subdirectory must supply rules for building sources it contributes
Src/PuttyInterface/%.o: ../Src/PuttyInterface/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F302xC -I"D:/Documents/STM/Robot/motorsTEST/Inc" -I"D:/Documents/STM/Robot/motorsTEST/Drivers/STM32F3xx_HAL_Driver/Inc" -I"D:/Documents/STM/Robot/motorsTEST/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM/Robot/motorsTEST/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"D:/Documents/STM/Robot/motorsTEST/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


