################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/comm_lora.c \
../Driver/comm_mavlink.c 

C_DEPS += \
./Driver/comm_lora.d \
./Driver/comm_mavlink.d 

OBJS += \
./Driver/comm_lora.o \
./Driver/comm_mavlink.o 

DIR_OBJS += \
./Driver/*.o \

DIR_DEPS += \
./Driver/*.d \

DIR_EXPANDS += \
./Driver/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Driver/%.o: ../Driver/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -DCore_V5F -I"d:/CH32H417/EVT/EXAM/SRC/Debug" -I"d:/CH32H417/EVT/EXAM/SRC/Core" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/V5F/User" -I"d:/CH32H417/EVT/EXAM/SRC/Peripheral/inc" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/Common" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/V5F/Driver" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/Middleware/MAVLink" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

