################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/IMU_Sensor/MahonyAHRS.c \
../Core/Src/IMU_Sensor/acc_gyro_calc.c \
../Core/Src/IMU_Sensor/mpu9250.c 

OBJS += \
./Core/Src/IMU_Sensor/MahonyAHRS.o \
./Core/Src/IMU_Sensor/acc_gyro_calc.o \
./Core/Src/IMU_Sensor/mpu9250.o 

C_DEPS += \
./Core/Src/IMU_Sensor/MahonyAHRS.d \
./Core/Src/IMU_Sensor/acc_gyro_calc.d \
./Core/Src/IMU_Sensor/mpu9250.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/IMU_Sensor/%.o Core/Src/IMU_Sensor/%.su Core/Src/IMU_Sensor/%.cyclo: ../Core/Src/IMU_Sensor/%.c Core/Src/IMU_Sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I"C:/Users/Uzeyir Varli/Desktop/quadcopter/STM32F072RB_QUADCOPTER/Core/Inc/Hardware" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-IMU_Sensor

clean-Core-2f-Src-2f-IMU_Sensor:
	-$(RM) ./Core/Src/IMU_Sensor/MahonyAHRS.cyclo ./Core/Src/IMU_Sensor/MahonyAHRS.d ./Core/Src/IMU_Sensor/MahonyAHRS.o ./Core/Src/IMU_Sensor/MahonyAHRS.su ./Core/Src/IMU_Sensor/acc_gyro_calc.cyclo ./Core/Src/IMU_Sensor/acc_gyro_calc.d ./Core/Src/IMU_Sensor/acc_gyro_calc.o ./Core/Src/IMU_Sensor/acc_gyro_calc.su ./Core/Src/IMU_Sensor/mpu9250.cyclo ./Core/Src/IMU_Sensor/mpu9250.d ./Core/Src/IMU_Sensor/mpu9250.o ./Core/Src/IMU_Sensor/mpu9250.su

.PHONY: clean-Core-2f-Src-2f-IMU_Sensor

