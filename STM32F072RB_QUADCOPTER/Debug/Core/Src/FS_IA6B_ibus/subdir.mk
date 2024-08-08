################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/FS_IA6B_ibus/FS-IA6B_ibus.c 

OBJS += \
./Core/Src/FS_IA6B_ibus/FS-IA6B_ibus.o 

C_DEPS += \
./Core/Src/FS_IA6B_ibus/FS-IA6B_ibus.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/FS_IA6B_ibus/%.o Core/Src/FS_IA6B_ibus/%.su Core/Src/FS_IA6B_ibus/%.cyclo: ../Core/Src/FS_IA6B_ibus/%.c Core/Src/FS_IA6B_ibus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F072xB -c -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I"C:/Users/Uzeyir Varli/Desktop/quadcopter/STM32F072RB_QUADCOPTER/Core/Inc/Hardware" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-FS_IA6B_ibus

clean-Core-2f-Src-2f-FS_IA6B_ibus:
	-$(RM) ./Core/Src/FS_IA6B_ibus/FS-IA6B_ibus.cyclo ./Core/Src/FS_IA6B_ibus/FS-IA6B_ibus.d ./Core/Src/FS_IA6B_ibus/FS-IA6B_ibus.o ./Core/Src/FS_IA6B_ibus/FS-IA6B_ibus.su

.PHONY: clean-Core-2f-Src-2f-FS_IA6B_ibus

