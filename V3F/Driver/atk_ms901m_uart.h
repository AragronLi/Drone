/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : atk_ms901m_uart.h
 * 作者                : WCH
 * 版本                : V1.0.0
 * 日期                : 2025/05/28
 * 描述                : ATK-MS901M 十轴 IMU 串口接口层头文件
 *                       USART3: PB10(TX) + PB11(RX), AF7, HB1 总线
 *********************************************************************************
 * Copyright (c) 2025 南京沁恒微电子股份有限公司
 *******************************************************************************/

#ifndef __ATK_MS901M_UART_H
#define __ATK_MS901M_UART_H

#include "ch32h417.h"

/* ---- USART3 引脚映射（符合飞控系统架构） ---- */
#define ATK_MS901M_UART                 USART3
#define ATK_MS901M_UART_IRQn            USART3_IRQn

#define ATK_MS901M_UART_TX_PORT         GPIOB
#define ATK_MS901M_UART_TX_PIN          GPIO_Pin_10
#define ATK_MS901M_UART_TX_PIN_SRC      GPIO_PinSource10

#define ATK_MS901M_UART_RX_PORT         GPIOB
#define ATK_MS901M_UART_RX_PIN          GPIO_Pin_11
#define ATK_MS901M_UART_RX_PIN_SRC      GPIO_PinSource11

#define ATK_MS901M_UART_GPIO_AF         GPIO_AF7

/* ATK-MS901M 出厂默认波特率 = 115200
   通过上位机配置后可提升至 460800（飞控目标速率） */
#define ATK_MS901M_UART_BAUDRATE        115200

/* 接收环形 FIFO 缓冲区大小 */
#define ATK_MS901M_UART_RX_FIFO_SIZE    256

/* ---- 公开接口 ---- */
uint8_t  atk_ms901m_uart_rx_fifo_write(uint8_t *dat, uint16_t len);
uint16_t atk_ms901m_uart_rx_fifo_read(uint8_t *dat, uint16_t len);
void     atk_ms901m_uart_rx_fifo_flush(void);
void     atk_ms901m_uart_send(uint8_t *dat, uint8_t len);
void     atk_ms901m_uart_init(uint32_t baudrate);
void     ATK_MS901M_UART_IRQHandler(void);

#endif
