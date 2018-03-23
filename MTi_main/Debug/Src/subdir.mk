################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MTiControl.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/spi.c \
../Src/stm32f3xx_hal_msp.c \
../Src/stm32f3xx_it.c \
../Src/system_stm32f3xx.c \
../Src/tim.c \
../Src/usart.c \
../Src/xbusmessage.c \
../Src/xbusparser.c \
../Src/xbusutility.c \
../Src/xsdeviceid.c 

OBJS += \
./Src/MTiControl.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/spi.o \
./Src/stm32f3xx_hal_msp.o \
./Src/stm32f3xx_it.o \
./Src/system_stm32f3xx.o \
./Src/tim.o \
./Src/usart.o \
./Src/xbusmessage.o \
./Src/xbusparser.o \
./Src/xbusutility.o \
./Src/xsdeviceid.o 

C_DEPS += \
./Src/MTiControl.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/spi.d \
./Src/stm32f3xx_hal_msp.d \
./Src/stm32f3xx_it.d \
./Src/system_stm32f3xx.d \
./Src/tim.d \
./Src/usart.d \
./Src/xbusmessage.d \
./Src/xbusparser.d \
./Src/xbusutility.d \
./Src/xsdeviceid.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F302xC -I"D:/Documents/STM/Robot/MTi_main/Inc" -I"D:/Documents/STM/Robot/MTi_main/Drivers/STM32F3xx_HAL_Driver/Inc" -I"D:/Documents/STM/Robot/MTi_main/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"D:/Documents/STM/Robot/MTi_main/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"D:/Documents/STM/Robot/MTi_main/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


