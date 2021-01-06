################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Nobita_UART_NEW.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Nobita_UART_NEW.list \

OBJCOPY_BIN += \
Nobita_UART_NEW.bin \


# All Target
all: Nobita_UART_NEW.elf secondary-outputs

# Tool invocations
Nobita_UART_NEW.elf: $(OBJS) $(USER_OBJS) /home/pete29480/STM32CubeIDE/workspace_1.4.0/Nobita_UART_NEW/STM32F411RETX_FLASH.ld
	arm-none-eabi-gcc -o "Nobita_UART_NEW.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/pete29480/STM32CubeIDE/workspace_1.4.0/Nobita_UART_NEW/STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Nobita_UART_NEW.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Nobita_UART_NEW.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Nobita_UART_NEW.list"
	@echo 'Finished building: $@'
	@echo ' '

Nobita_UART_NEW.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Nobita_UART_NEW.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
