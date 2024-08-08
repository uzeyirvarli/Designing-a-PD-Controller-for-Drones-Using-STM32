################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Hardware/adc.c \
../Core/Src/Hardware/gpio.c \
../Core/Src/Hardware/hardware.c \
../Core/Src/Hardware/i2c.c \
../Core/Src/Hardware/spi.c \
../Core/Src/Hardware/system_clock_config.c \
../Core/Src/Hardware/tim.c \
../Core/Src/Hardware/usart.c 

OBJS += \
./Core/Src/Hardware/adc.o \
./Core/Src/Hardware/gpio.o \
./Core/Src/Hardware/hardware.o \
./Core/Src/Hardware/i2c.o \
./Core/Src/Hardware/spi.o \
./Core/Src/Hardware/system_clock_config.o \
./Core/Src/Hardware/tim.o \
./Core/Src/Hardware/usart.o 

C_DEPS += \
./Core/Src/Hardware/adc.d \
./Core/Src/Hardware/gpio.d \
./Core/Src/Hardware/hardware.d \
./Core/Src/Hardware/i2c.d \
./Core/Src/Hardware/spi.d \
./Core/Src/Hardware/system_clock_config.d \
./Core/Src/Hardware/tim.d \
./Core/Src/Hardware/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Hardware/%.o Core/Src/Hardware/%.su Core/Src/Hardware/%.cyclo: ../Core/Src/Hardware/%.c Core/Src/Hardware/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I"C:/Users/Uzeyir Varli/Desktop/quadcopter/STM32F072RB_QUADCOPTER/Core/Inc/Hardware" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Hardware

clean-Core-2f-Src-2f-Hardware:
	-$(RM) ./Core/Src/Hardware/adc.cyclo ./Core/Src/Hardware/adc.d ./Core/Src/Hardware/adc.o ./Core/Src/Hardware/adc.su ./Core/Src/Hardware/gpio.cyclo ./Core/Src/Hardware/gpio.d ./Core/Src/Hardware/gpio.o ./Core/Src/Hardware/gpio.su ./Core/Src/Hardware/hardware.cyclo ./Core/Src/Hardware/hardware.d ./Core/Src/Hardware/hardware.o ./Core/Src/Hardware/hardware.su ./Core/Src/Hardware/i2c.cyclo ./Core/Src/Hardware/i2c.d ./Core/Src/Hardware/i2c.o ./Core/Src/Hardware/i2c.su ./Core/Src/Hardware/spi.cyclo ./Core/Src/Hardware/spi.d ./Core/Src/Hardware/spi.o ./Core/Src/Hardware/spi.su ./Core/Src/Hardware/system_clock_config.cyclo ./Core/Src/Hardware/system_clock_config.d ./Core/Src/Hardware/system_clock_config.o ./Core/Src/Hardware/system_clock_config.su ./Core/Src/Hardware/tim.cyclo ./Core/Src/Hardware/tim.d ./Core/Src/Hardware/tim.o ./Core/Src/Hardware/tim.su ./Core/Src/Hardware/usart.cyclo ./Core/Src/Hardware/usart.d ./Core/Src/Hardware/usart.o ./Core/Src/Hardware/usart.su

.PHONY: clean-Core-2f-Src-2f-Hardware

