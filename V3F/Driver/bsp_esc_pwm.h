/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : bsp_esc_pwm.h
 * 作者                : WCH
 * 版本                : V1.0.0
 * 日期                : 2025/05/28
 * 描述                : ESC PWM 驱动头文件
 *                      TIM1 CH1-4: PA8-11, AF1, HB2 总线
 *                      50Hz / 1000-2000μs 标准 ESC 信号
 *********************************************************************************
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 *******************************************************************************/

#ifndef __BSP_ESC_PWM_H
#define __BSP_ESC_PWM_H

#include "ch32h417.h"

/* 电机索引 */
#define ESC_MOTOR1   0
#define ESC_MOTOR2   1
#define ESC_MOTOR3   2
#define ESC_MOTOR4   3

/* PWM 参数 */
#define ESC_PWM_FREQ_HZ       50
#define ESC_PWM_PERIOD_US     20000     /* 20ms 周期 */
#define ESC_PWM_MIN_US        1000      /* 停转 */
#define ESC_PWM_IDLE_US       1060      /* 怠速 */
#define ESC_PWM_MAX_US        2000      /* 全速 */

/* 全局电机 PWM 值: 1000~2000 us */
extern volatile uint16_t g_esc_pwm[4];

void bsp_esc_pwm_init(void);
void bsp_esc_pwm_set(uint8_t motor, uint16_t pulse_us);
void bsp_esc_pwm_set_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif
