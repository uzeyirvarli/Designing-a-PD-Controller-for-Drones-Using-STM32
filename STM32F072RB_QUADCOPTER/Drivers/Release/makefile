################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := QUADCOPTER_ROBUST_CONTROLLER
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
QUADCOPTER_ROBUST_CONTROLLER.elf \

MAP_FILES += \
QUADCOPTER_ROBUST_CONTROLLER.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
QUADCOPTER_ROBUST_CONTROLLER.list \


# All Target
all: main-build

# Main-build Target
main-build: QUADCOPTER_ROBUST_CONTROLLER.elf secondary-outputs

# Tool invocations
QUADCOPTER_ROBUST_CONTROLLER.elf QUADCOPTER_ROBUST_CONTROLLER.map: $(OBJS) $(USER_OBJS) C:\Users\uzeyi\OneDrive\Masaüstü\quadcopter\QUADCOPTER_ROBUST_CONTROLLER\STM32F072RBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "QUADCOPTER_ROBUST_CONTROLLER.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\uzeyi\OneDrive\Masaüstü\quadcopter\QUADCOPTER_ROBUST_CONTROLLER\STM32F072RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="QUADCOPTER_ROBUST_CONTROLLER.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

QUADCOPTER_ROBUST_CONTROLLER.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "QUADCOPTER_ROBUST_CONTROLLER.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) QUADCOPTER_ROBUST_CONTROLLER.elf QUADCOPTER_ROBUST_CONTROLLER.list QUADCOPTER_ROBUST_CONTROLLER.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
