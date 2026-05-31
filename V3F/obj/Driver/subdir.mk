################################################################################
# MRS Version: 2.4.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driver/atk_ms901m.c \
../Driver/atk_ms901m_uart.c \
../Driver/bsp_esc_pwm.c \
../Driver/bsp_tof_tfmini.c \
../Driver/oled_ssd1306.c \
../Driver/sensor_compass.c \
../Driver/sensor_flow.c \
../Driver/sensor_gps.c 

C_DEPS += \
./Driver/atk_ms901m.d \
./Driver/atk_ms901m_uart.d \
./Driver/bsp_esc_pwm.d \
./Driver/bsp_tof_tfmini.d \
./Driver/oled_ssd1306.d \
./Driver/sensor_compass.d \
./Driver/sensor_flow.d \
./Driver/sensor_gps.d 

OBJS += \
./Driver/atk_ms901m.o \
./Driver/atk_ms901m_uart.o \
./Driver/bsp_esc_pwm.o \
./Driver/bsp_tof_tfmini.o \
./Driver/oled_ssd1306.o \
./Driver/sensor_compass.o \
./Driver/sensor_flow.o \
./Driver/sensor_gps.o 

DIR_OBJS += \
./Driver/*.o \

DIR_DEPS += \
./Driver/*.d \

DIR_EXPANDS += \
./Driver/*.253r.expand \


# Each subdirectory must supply rules for building sources it contributes
Driver/%.o: ../Driver/%.c
	@	riscv-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -DCore_V3F -I"d:/CH32H417/EVT/EXAM/SRC/Debug" -I"d:/CH32H417/EVT/EXAM/SRC/Core" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/V3F/User" -I"d:/CH32H417/EVT/EXAM/SRC/Peripheral/inc" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/Common" -I"d:/CH32H417/EVT/EXAM/USART/USART_Interrupt/V3F/Driver" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

