/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : comm_lora.h
 * 描述                : ATK LoRa 数传模块 USART8 驱动头文件
 *                      USART8: PE8(TX) + PE7(RX), AF7, HB1 总线
 *                      波特率 115200, 8N1
 *******************************************************************************/

#ifndef __COMM_LORA_H
#define __COMM_LORA_H

#include "ch32h417.h"

/* ---- USART8 引脚映射 ---- */
#define COMM_LORA_UART              USART8
#define COMM_LORA_UART_IRQn         USART8_IRQn

#define COMM_LORA_UART_TX_PORT      GPIOE
#define COMM_LORA_UART_TX_PIN       GPIO_Pin_8
#define COMM_LORA_UART_TX_PIN_SRC   GPIO_PinSource8

#define COMM_LORA_UART_RX_PORT      GPIOE
#define COMM_LORA_UART_RX_PIN       GPIO_Pin_7
#define COMM_LORA_UART_RX_PIN_SRC   GPIO_PinSource7

#define COMM_LORA_UART_GPIO_AF      GPIO_AF7

#define COMM_LORA_UART_BAUDRATE     115200

/* 接收环形 FIFO 缓冲区大小 */
#define COMM_LORA_UART_RX_FIFO_SIZE 512

/* ---- 公开接口 ---- */
void     comm_lora_init(uint32_t baudrate);
void     comm_lora_send_bytes(const uint8_t *dat, uint16_t len);
uint16_t comm_lora_available(void);
uint8_t  comm_lora_read_byte(void);
void     comm_lora_flush(void);
void     COMM_LORA_UART_IRQHandler(void);

#endif /* __COMM_LORA_H */
