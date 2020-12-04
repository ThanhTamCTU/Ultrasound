################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
Ultrasoundsensor_stm32_LuanVan_pj.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Ultrasoundsensor_stm32_LuanVan_pj.list \

OBJCOPY_BIN += \
Ultrasoundsensor_stm32_LuanVan_pj.bin \


# All Target
all: Ultrasoundsensor_stm32_LuanVan_pj.elf secondary-outputs

# Tool invocations
Ultrasoundsensor_stm32_LuanVan_pj.elf: $(OBJS) $(USER_OBJS) E:\Tai_Lieu\LUAN\ VAN\Code\ embeded\Ultrasound_sensor_Workspace\Ultrasoundsensor_stm32_LuanVan_pj\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "Ultrasoundsensor_stm32_LuanVan_pj.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"E:\Tai_Lieu\LUAN VAN\Code embeded\Ultrasound_sensor_Workspace\Ultrasoundsensor_stm32_LuanVan_pj\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Ultrasoundsensor_stm32_LuanVan_pj.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Ultrasoundsensor_stm32_LuanVan_pj.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Ultrasoundsensor_stm32_LuanVan_pj.list"
	@echo 'Finished building: $@'
	@echo ' '

Ultrasoundsensor_stm32_LuanVan_pj.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Ultrasoundsensor_stm32_LuanVan_pj.bin"
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
