/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : bsp_tof_tfmini.h
 * 描述                : TFmini Plus / VI530x 激光测距模块驱动
 *                       USART6 RX: PC11, AF8, HB1 总线
 *
 * 帧格式 (6 字节):
 *   BB 03 DistL DistH Conf CRC
 *   距离 mm (小端), 可信度 0-100, CRC 从头累加
 *******************************************************************************/
#ifndef __BSP_TOF_TFMINI_H
#define __BSP_TOF_TFMINI_H

#include "ch32h417.h"

typedef struct {
    uint16_t distance_mm;
    uint8_t  confidence;
    uint8_t  updated;
} tfmini_data_t;

extern volatile uint32_t g_tof_irq_count;
extern volatile uint32_t g_tof_frame_count;
extern volatile uint32_t g_tof_crc_error_count;
extern volatile uint32_t g_tof_len_error_count;
extern volatile uint8_t  g_tof_last_rx_byte;
extern volatile uint8_t  g_tof_last_frame[6];
extern volatile uint32_t g_tof_poll_count;
extern volatile uint8_t  g_tof_poll_last_byte;

void bsp_tof_tfmini_init(void);
void bsp_tof_tfmini_poll(tfmini_data_t *data);
void USART6_IRQHandler(void);

#endif
