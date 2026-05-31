/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : sensor_flow.h
 * 描述                : ATK-PMW3901 光流传感器 SPI 驱动
 *                       接口: SPI3 (PB3/SCK, PB4/MISO, PB5/MOSI, PC5/CS)
 *                       速率: ~2MHz, 100Hz 轮询
 *
 * 高度来源: TFmini 下视 TOF (USART6/PC11), 不使用 VL53LXX
 * 参考: ATK-PMW3901 光流模块用户手册
 *******************************************************************************/
#ifndef __SENSOR_FLOW_H
#define __SENSOR_FLOW_H

#include "ch32h417.h"
#include <stdbool.h>

/* ---- PMW3901 寄存器 ---- */
#define PMW3901_REG_PRODUCT_ID      0x00
#define PMW3901_REG_MOTION          0x02
#define PMW3901_REG_DELTA_X_L       0x03
#define PMW3901_REG_DELTA_X_H       0x04
#define PMW3901_REG_DELTA_Y_L       0x05
#define PMW3901_REG_DELTA_Y_H       0x06
#define PMW3901_REG_SQUAL           0x07
#define PMW3901_REG_RAW_DATA_SUM    0x08
#define PMW3901_REG_MAX_RAW_DATA    0x09
#define PMW3901_REG_MIN_RAW_DATA    0x0A
#define PMW3901_REG_SHUTTER_L       0x0B
#define PMW3901_REG_SHUTTER_H       0x0C
#define PMW3901_REG_MOTION_BURST    0x16
#define PMW3901_REG_POWER_UP_RESET  0x3A
#define PMW3901_REG_INV_PRODUCT_ID  0x5F

/* ---- 参数宏 ---- */
#define RESOLUTION     0.2131946f   /* 1m 高度下 1 像素对应位移 (cm) */
#define OUTLIER_LIMIT  50           /* 像素野值限幅 */
#define VEL_LIMIT      200.0f       /* 速度限幅 (cm/s) */
#define COMP_COEFF     480.0f       /* 倾角补偿系数 */

/* ---- 12 字节运动数据突发读取 ---- */
typedef struct __attribute__((packed)) motionBurst_s {
    union {
        uint8_t motion;
        struct {
            uint8_t frameFrom0    : 1;
            uint8_t runMode       : 2;
            uint8_t reserved1     : 1;
            uint8_t rawFrom0      : 1;
            uint8_t reserved2     : 2;
            uint8_t motionOccured : 1;
        };
    };
    uint8_t  observation;
    int16_t  deltaX;
    int16_t  deltaY;
    uint8_t  squal;
    uint8_t  rawDataSum;
    uint8_t  maxRawData;
    uint8_t  minRawData;
    uint16_t shutter;
} motionBurst_t;

/* ---- 光流状态 ---- */
typedef struct {
    float pixSum[2];       /* 累积像素 (自起飞) */
    float pixComp[2];      /* 倾角补偿像素 */
    float pixValid[2];     /* 有效像素 (补偿后) */
    float pixValidLast[2]; /* 上一帧有效像素 */
    float deltaPos[2];     /* 帧间位移 (cm) */
    float deltaVel[2];     /* 瞬时速度 (cm/s) */
    float posSum[2];       /* 累积位移 (cm) */
    float velLpf[2];       /* 低通速度 (cm/s) */
    bool  isOpFlowOk;      /* 光流是否正常工作 */
    bool  isDataValid;     /* 数据有效 (高度范围内) */
} opFlow_t;

/* ---- 公开 API ---- */
void sensor_flow_init(void);
void sensor_flow_poll(opFlow_t *flow, float height_m,
                      float roll_rad, float pitch_rad, float dt);
void sensor_flow_reset(opFlow_t *flow);

#endif /* __SENSOR_FLOW_H */
