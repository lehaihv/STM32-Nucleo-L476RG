################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
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
oled_adc.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
oled_adc.list \

OBJCOPY_BIN += \
oled_adc.bin \


# All Target
all: oled_adc.elf secondary-outputs

# Tool invocations
oled_adc.elf: $(OBJS) $(USER_OBJS) C:\Users\haihv\Documents\GitHub\STM32-Nucleo-L476RG\oled_adc_GitHub\STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "oled_adc.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\haihv\Documents\GitHub\STM32-Nucleo-L476RG\oled_adc_GitHub\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="oled_adc.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

oled_adc.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "oled_adc.list"
	@echo 'Finished building: $@'
	@echo ' '

oled_adc.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "oled_adc.bin"
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
