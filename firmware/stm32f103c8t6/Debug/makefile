################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := gripForceMeasurement32x64UsingSTM32f103C8T6_main
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
gripForceMeasurement32x64UsingSTM32f103C8T6_main.elf \

MAP_FILES += \
gripForceMeasurement32x64UsingSTM32f103C8T6_main.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
gripForceMeasurement32x64UsingSTM32f103C8T6_main.list \

OBJCOPY_HEX += \
gripForceMeasurement32x64UsingSTM32f103C8T6_main.hex \

OBJCOPY_BIN += \
gripForceMeasurement32x64UsingSTM32f103C8T6_main.bin \


# All Target
all: main-build

# Main-build Target
main-build: gripForceMeasurement32x64UsingSTM32f103C8T6_main.elf secondary-outputs

# Tool invocations
gripForceMeasurement32x64UsingSTM32f103C8T6_main.elf gripForceMeasurement32x64UsingSTM32f103C8T6_main.map: $(OBJS) $(USER_OBJS) E:\Personal\Study\CoOp\ Work\Grip\ Force\ Device\Complete\ Design\firmware\stm32f103c8t6\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "gripForceMeasurement32x64UsingSTM32f103C8T6_main.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"E:\Personal\Study\CoOp Work\Grip Force Device\Complete Design\firmware\stm32f103c8t6\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="gripForceMeasurement32x64UsingSTM32f103C8T6_main.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

gripForceMeasurement32x64UsingSTM32f103C8T6_main.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "gripForceMeasurement32x64UsingSTM32f103C8T6_main.list"
	@echo 'Finished building: $@'
	@echo ' '

gripForceMeasurement32x64UsingSTM32f103C8T6_main.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "gripForceMeasurement32x64UsingSTM32f103C8T6_main.hex"
	@echo 'Finished building: $@'
	@echo ' '

gripForceMeasurement32x64UsingSTM32f103C8T6_main.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "gripForceMeasurement32x64UsingSTM32f103C8T6_main.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout gripForceMeasurement32x64UsingSTM32f103C8T6_main.bin gripForceMeasurement32x64UsingSTM32f103C8T6_main.elf gripForceMeasurement32x64UsingSTM32f103C8T6_main.hex gripForceMeasurement32x64UsingSTM32f103C8T6_main.list gripForceMeasurement32x64UsingSTM32f103C8T6_main.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
