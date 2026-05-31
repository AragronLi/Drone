/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : sensor_gps.h
 * 描述                : SR2525M10D GPS NMEA0183 驱动
 *                       USART4: PC6/PC7, 默认 38400-8N1
 *******************************************************************************/
#ifndef __SENSOR_GPS_H
#define __SENSOR_GPS_H

#include "ch32h417.h"

/* ---- USART4 引脚映射 (SR2525M10D TXD/RXD) ---- */
#define SENSOR_GPS_UART                 USART4
#define SENSOR_GPS_UART_IRQn            USART4_IRQn
#define SENSOR_GPS_UART_BAUDRATE        38400

#define SENSOR_GPS_UART_TX_PORT         GPIOC
#define SENSOR_GPS_UART_TX_PIN          GPIO_Pin_6
#define SENSOR_GPS_UART_TX_PIN_SRC      GPIO_PinSource6

#define SENSOR_GPS_UART_RX_PORT         GPIOC
#define SENSOR_GPS_UART_RX_PIN          GPIO_Pin_7
#define SENSOR_GPS_UART_RX_PIN_SRC      GPIO_PinSource7

#define SENSOR_GPS_UART_GPIO_AF         GPIO_AF7

#define SENSOR_GPS_RX_FIFO_SIZE         1024
#define SENSOR_GPS_NMEA_LINE_MAX        128

/* ---- GPS 数据结构 ---- */
typedef struct {
    volatile uint32_t irq_count;          /* USART4 RXNE 中断计数 */
    volatile uint32_t poll_count;         /* 轮询兜底读到的字节数 */
    volatile uint8_t  last_rx_byte;       /* 最近收到字节 */
    volatile uint16_t fifo_level;         /* 当前 FIFO 可读字节数 */
    volatile uint8_t  rxne_flag;          /* 最近一次 poll/init 看到的 RXNE 标志 */
    volatile uint8_t  ore_flag;           /* 最近一次 poll/init 看到的 ORE 标志 */
    volatile uint8_t  fe_flag;            /* 最近一次 poll/init 看到的 FE 标志 */
    volatile uint8_t  ne_flag;            /* 最近一次 poll/init 看到的 NE 标志 */

    uint8_t  fix_valid;             /* RMC/GLL 状态 A=1 */
    uint8_t  fix_quality;           /* GGA: 0=无效, 1=GPS, 2=DGPS/SBAS, 4/5=RTK */
    uint8_t  fix_type;              /* GSA: 1=无定位, 2=2D, 3=3D */
    uint8_t  satellites_used;       /* GGA 使用卫星数 */
    uint8_t  satellites_visible;    /* GSV 可见卫星数 */

    int32_t  latitude_deg_e7;       /* 纬度, 度 * 1e7, 北正南负 */
    int32_t  longitude_deg_e7;      /* 经度, 度 * 1e7, 东正西负 */
    int32_t  altitude_mm;           /* 海拔高度, mm */
    int32_t  geoid_separation_mm;   /* 大地水准面差, mm */

    uint16_t hdop_x100;
    uint16_t pdop_x100;
    uint16_t vdop_x100;

    uint32_t speed_cms;             /* 地速, cm/s */
    uint16_t course_deg_x100;       /* 地速航向, deg * 100 */
    uint8_t  course_valid;

    uint32_t utc_time_ms;           /* 当天 UTC 毫秒 */
    uint32_t date_ddmmyy;           /* RMC 日期 ddmmyy */
    uint32_t sentence_count;
    uint32_t checksum_ok_count;
    uint32_t checksum_error_count;
    char     last_sentence[6];      /* 如 GGA/RMC/VTG */
    uint8_t  updated;
} gps_data_t;

void sensor_gps_init(uint32_t baudrate);
void sensor_gps_poll(void);
uint8_t sensor_gps_get_data(gps_data_t *data);
void SENSOR_GPS_UART_IRQHandler(void);

#endif /* __SENSOR_GPS_H */
