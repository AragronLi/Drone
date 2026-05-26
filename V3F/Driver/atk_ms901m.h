/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : atk_ms901m.h
 * 作者                : WCH
 * 版本                : V1.0.0
 * 日期                : 2025/05/28
 * 描述                : ATK-MS901M 十轴 IMU 模块协议层头文件
 *                      传感器: ICM-20602 (6轴陀螺仪+加速度计)
 *                              AK8963C (3轴磁力计)
 *                              SPL06-001 (气压计)
 *                      通信: UART 自定义二进制协议
 *                      帧格式: 0x55 + 帧头高位 + ID + 长度 + 数据[0..27] + 校验和
 *********************************************************************************
 * Copyright (c) 2025 南京沁恒微电子股份有限公司
 *******************************************************************************/

#ifndef __ATK_MS901M_H
#define __ATK_MS901M_H

#include "ch32h417.h"

/* 帧数据最大字节数 */
#define ATK_MS901M_FRAME_DAT_MAX_SIZE        28

/* ---- 上传帧 ID（模块 → MCU） ---- */
#define ATK_MS901M_FRAME_ID_ATTITUDE         0x01    /* 姿态角: 横滚/俯仰/偏航 */
#define ATK_MS901M_FRAME_ID_QUAT             0x02    /* 四元数: q0~q3 */
#define ATK_MS901M_FRAME_ID_GYRO_ACCE        0x03    /* 陀螺仪 + 加速度计 */
#define ATK_MS901M_FRAME_ID_MAG              0x04    /* 磁力计 */
#define ATK_MS901M_FRAME_ID_BARO             0x05    /* 气压计 */
#define ATK_MS901M_FRAME_ID_PORT             0x06    /* 端口数据 D0~D3 */

/* ---- 配置寄存器帧 ID（MCU → 模块） ---- */
#define ATK_MS901M_REG_ID_GYROFSR            0x03    /* 陀螺仪量程: 0=250,1=500,2=1000,3=2000 dps */
#define ATK_MS901M_REG_ID_ACCFSR             0x04    /* 加速度计量程: 0=2,1=4,2=8,3=16 G */
#define ATK_MS901M_REG_ID_RETURNRATE         0x0A    /* 回传频率: 0=0.1,1=0.2,2=0.5,3=1,4=2,5=5,6=10,7=20,8=50,9=100,10=200Hz */
#define ATK_MS901M_REG_ID_RETURNSET          0x08    /* 回传内容设置. Bit0=姿态,Bit1=四元数,Bit2=陀螺/加计,Bit3=磁力计,Bit4=气压计,Bit5=端口 */
#define ATK_MS901M_REG_ID_BAUD               0x07    /* 波特率: 0=4800,1=9600,2=19200,3=38400,4=57600,5=115200,6=230400,7=460800,8=921600 */

/* ---- 帧 ID 类型 ---- */
#define ATK_MS901M_FRAME_ID_TYPE_UPLOAD      0       /* 上传帧: 0x55 + 0x55 */
#define ATK_MS901M_FRAME_ID_TYPE_ACK         1       /* 应答帧: 0x55 + 0xAF */

/* ---- 错误码 ---- */
#define ATK_MS901M_EOK                       0       /* 无错误 */
#define ATK_MS901M_ERROR                     1       /* 通用错误 */
#define ATK_MS901M_EINVAL                    2       /* 参数无效 */
#define ATK_MS901M_ETIMEOUT                  3       /* 超时 */

/* ---- 数据结构体 ---- */

/* 姿态角 */
typedef struct {
    float roll;     /* 横滚角 ° */
    float pitch;    /* 俯仰角 ° */
    float yaw;      /* 偏航角 ° */
} atk_ms901m_attitude_data_t;

/* 四元数 */
typedef struct {
    float q0, q1, q2, q3;
} atk_ms901m_quaternion_data_t;

/* 陀螺仪 */
typedef struct {
    struct { int16_t x, y, z; } raw;  /* 原始 ADC 值 */
    float x, y, z;                    /* 角速率 dps */
} atk_ms901m_gyro_data_t;

/* 加速度计 */
typedef struct {
    struct { int16_t x, y, z; } raw;  /* 原始 ADC 值 */
    float x, y, z;                    /* 加速度 G */
} atk_ms901m_accelerometer_data_t;

/* 磁力计 */
typedef struct {
    int16_t x, y, z;          /* 磁场强度 */
    float temperature;        /* 温度 °C */
} atk_ms901m_magnetometer_data_t;

/* 气压计 */
typedef struct {
    int32_t pressure;         /* 气压 Pa */
    int32_t altitude;         /* 海拔 cm */
    float   temperature;      /* 温度 °C */
} atk_ms901m_barometer_data_t;

/* 端口数据 */
typedef struct {
    uint16_t d0, d1, d2, d3;
} atk_ms901m_port_data_t;

/* ---- 公开接口 ---- */
uint8_t atk_ms901m_init(uint32_t baudrate);
uint8_t atk_ms901m_get_attitude(atk_ms901m_attitude_data_t *dat, uint32_t timeout);
uint8_t atk_ms901m_get_quaternion(atk_ms901m_quaternion_data_t *dat, uint32_t timeout);
uint8_t atk_ms901m_get_gyro_accelerometer(atk_ms901m_gyro_data_t *gyro_dat, atk_ms901m_accelerometer_data_t *accel_dat, uint32_t timeout);
uint8_t atk_ms901m_get_magnetometer(atk_ms901m_magnetometer_data_t *dat, uint32_t timeout);
uint8_t atk_ms901m_get_barometer(atk_ms901m_barometer_data_t *dat, uint32_t timeout);
uint8_t atk_ms901m_get_port(atk_ms901m_port_data_t *dat, uint32_t timeout);

#endif
