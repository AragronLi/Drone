################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include Common/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
USART_Interrupt_V5F.bin \
USART_Interrupt_V5F.hex \

SECONDARY_LIST += \
USART_Interrupt_V5F.lst \

SECONDARY_SIZE += \
USART_Interrupt_V5F.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: USART_Interrupt_V5F.elf secondary-outputs

# Tool invocations
USART_Interrupt_V5F.elf: $(OBJS) $(USER_OBJS_ESCAPE)
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -T "d:\CH32H417\EVT\EXAM\SRC\Ld\V5F\Link_v5f.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"USART_Interrupt_V5F.map" --specs=nano.specs --specs=nosys.specs -o "USART_Interrupt_V5F.elf" $(OBJS) $(LIBS)
USART_Interrupt_V5F.bin: USART_Interrupt_V5F.elf
	@	riscv-wch-elf-objcopy -O binary "USART_Interrupt_V5F.elf" "USART_Interrupt_V5F.bin"
USART_Interrupt_V5F.hex: USART_Interrupt_V5F.elf
	@	riscv-wch-elf-objcopy -O ihex "USART_Interrupt_V5F.elf" "USART_Interrupt_V5F.hex"
USART_Interrupt_V5F.lst: USART_Interrupt_V5F.elf
	@	riscv-wch-elf-objdump --all-headers --demangle --disassemble -M xw "USART_Interrupt_V5F.elf" > "USART_Interrupt_V5F.lst"
USART_Interrupt_V5F.siz: USART_Interrupt_V5F.elf
	riscv-wch-elf-size --format=berkeley "USART_Interrupt_V5F.elf"

# Other Targets
clean:
	-$(RM) $(DIR_OBJS) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE) USART_Interrupt_V5F.elf
	-$(RM) $(DIR_EXPANDS) $(CALLGRAPH_DOT)
	-$(RM) $(DIR_DEPS)
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets