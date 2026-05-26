/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : bsp_esc_pwm.c
 * 作者                : WCH
 * 版本                : V1.0.0
 * 日期                : 2025/05/28
 * 描述                : ESC PWM 驱动实现 (参照官方 TIM/PWM_Output 样例)
 *                      TIM1 CH1-4: PA8-11, AF1, HB2 总线
 *                      50Hz: TIM1_CLK=100MHz / PSC=100 = 1MHz / ARR=20000
 *                      脉宽: 1000~2000 (1~2ms), 1μs 分辨率
 *********************************************************************************
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 *******************************************************************************/

#include "bsp_esc_pwm.h"

/* 全局电机 PWM 值: 1000~2000 us */
volatile uint16_t g_esc_pwm[4] = {
    ESC_PWM_MIN_US,
    ESC_PWM_MIN_US,
    ESC_PWM_MIN_US,
    ESC_PWM_MIN_US
};

/*********************************************************************
 * @fn      bsp_esc_pwm_init
 * @brief   初始化 TIM1 四路 PWM 输出, 50Hz ESC 标准信号
 *          参照官方 TIM/PWM_Output 样例的初始化顺序
 */
__attribute__((noinline)) void bsp_esc_pwm_init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_OCInitTypeDef       TIM_OCInitStructure = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA | RCC_HB2Periph_TIM1 |
                          RCC_HB2Periph_AFIO, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8,  GPIO_AF1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 |
                                    GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period            = 20000 - 1;   /* 1MHz / 20000 = 50Hz */
    TIM_TimeBaseInitStructure.TIM_Prescaler         = 100 - 1;     /* 100MHz / 100 = 1MHz */
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = ESC_PWM_MIN_US;

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

/*********************************************************************
 * @fn      bsp_esc_pwm_set
 * @brief   设置单个电机 PWM 脉宽
 * @param   motor   : ESC_MOTOR1 ~ ESC_MOTOR4
 *          pulse_us: 脉宽 1000~2000 us
 */
void bsp_esc_pwm_set(uint8_t motor, uint16_t pulse_us)
{
    if (motor > ESC_MOTOR4) return;
    if (pulse_us < ESC_PWM_MIN_US) pulse_us = ESC_PWM_MIN_US;
    if (pulse_us > ESC_PWM_MAX_US) pulse_us = ESC_PWM_MAX_US;

    g_esc_pwm[motor] = pulse_us;

    switch (motor) {
    case ESC_MOTOR1: TIM_SetCompare1(TIM1, pulse_us); break;
    case ESC_MOTOR2: TIM_SetCompare2(TIM1, pulse_us); break;
    case ESC_MOTOR3: TIM_SetCompare3(TIM1, pulse_us); break;
    case ESC_MOTOR4: TIM_SetCompare4(TIM1, pulse_us); break;
    }
}

/*********************************************************************
 * @fn      bsp_esc_pwm_set_all
 * @brief   一次性设置四路电机 PWM 脉宽
 * @param   m1~m4: 四路脉宽 1000~2000 us
 */
void bsp_esc_pwm_set_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    g_esc_pwm[ESC_MOTOR1] = m1;
    g_esc_pwm[ESC_MOTOR2] = m2;
    g_esc_pwm[ESC_MOTOR3] = m3;
    g_esc_pwm[ESC_MOTOR4] = m4;

    TIM_SetCompare1(TIM1, m1);
    TIM_SetCompare2(TIM1, m2);
    TIM_SetCompare3(TIM1, m3);
    TIM_SetCompare4(TIM1, m4);
}
