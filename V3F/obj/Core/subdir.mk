################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
d:/CH32H417/EVT/EXAM/SRC/Core/core_riscv.c 

C_DEPS += \
./Core/core_riscv.d 

OBJS += \
./Core/core_riscv.o 

DIR_OBJS += \
./Core/*.o \

DIR_DEPS += \
./Core/*.d \

DIR_EXPANDS += \
./Core/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Core/core_riscv.o: d:/CH32H417/EVT/EXAM/SRC/Core/core_riscv.c
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -DCore_V3F -I"d:/CH32H417/EVT/EXAM/SRC/Debug" -I"d:/CH32H417/EVT/EXAM/SRC/Core" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/V3F/User" -I"d:/CH32H417/EVT/EXAM/SRC/Peripheral/inc" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/Common" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/V3F/Driver" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

