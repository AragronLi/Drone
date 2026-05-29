/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_it.h"
#include "comm_lora.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void IPC_CH0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  NVIC_SystemReset();
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      IPC_CH0_IRQHandler
 * @brief   V3F→V5F 传感器数据到达中断
 *          当前占位: 清除标志位 (FreeRTOS 就绪后改为 xSemaphoreGiveFromISR)
 */
void IPC_CH0_IRQHandler(void)
{
  if (IPC_GetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0) == SET) {
      IPC_ClearFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
  }
}

/*********************************************************************
 * @fn      USART7_IRQHandler
 * @brief   ATK LoRa 数传 USART7 中断入口, 转发到驱动层处理
 */
void USART7_IRQHandler(void)
{
    COMM_LORA_UART_IRQHandler();
}
