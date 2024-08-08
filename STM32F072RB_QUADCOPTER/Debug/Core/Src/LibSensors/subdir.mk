################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LibSensors/gyro.c \
../Core/Src/LibSensors/mpu9250.c 

OBJS += \
./Core/Src/LibSensors/gyro.o \
./Core/Src/LibSensors/mpu9250.o 

C_DEPS += \
./Core/Src/LibSensors/gyro.d \
./Core/Src/LibSensors/mpu9250.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LibSensors/%.o Core/Src/LibSensors/%.su Core/Src/LibSensors/%.cyclo: ../Core/Src/LibSensors/%.c Core/Src/LibSensors/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I"C:/Users/Uzeyir Varli/Desktop/quadcopter/Old Codes/STM32F072RB_QUADCOPTER/Core/Inc/Hardware" -I"C:/Users/Uzeyir Varli/Desktop/quadcopter/Old Codes/STM32F072RB_QUADCOPTER/Core/Inc/LibSensors" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LibSensors

clean-Core-2f-Src-2f-LibSensors:
	-$(RM) ./Core/Src/LibSensors/gyro.cyclo ./Core/Src/LibSensors/gyro.d ./Core/Src/LibSensors/gyro.o ./Core/Src/LibSensors/gyro.su ./Core/Src/LibSensors/mpu9250.cyclo ./Core/Src/LibSensors/mpu9250.d ./Core/Src/LibSensors/mpu9250.o ./Core/Src/LibSensors/mpu9250.su

.PHONY: clean-Core-2f-Src-2f-LibSensors

