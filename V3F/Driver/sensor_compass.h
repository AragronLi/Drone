/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : sensor_compass.h
 * 描述                : QMC5883L 罗盘传感器 I2C 驱动
 *                       I2C3: PA14(SCL) + PA13(SDA), AF7, 400kHz
 *                       器件地址: 0x0D (与OLED共享I2C3总线)
 *******************************************************************************/
#ifndef __SENSOR_COMPASS_H
#define __SENSOR_COMPASS_H

#include "ch32h417.h"

/* ---- QMC5883L I2C地址 ---- */
#define QMC5883L_ADDR       0x0D

/* ---- QMC5883L 寄存器 ---- */
#define QMC5883L_REG_DATA_X_L   0x00
#define QMC5883L_REG_DATA_X_H   0x01
#define QMC5883L_REG_DATA_Y_L   0x02
#define QMC5883L_REG_DATA_Y_H   0x03
#define QMC5883L_REG_DATA_Z_L   0x04
#define QMC5883L_REG_DATA_Z_H   0x05
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_TEMP_L     0x07
#define QMC5883L_REG_TEMP_H     0x08
#define QMC5883L_REG_CONTROL1   0x09
#define QMC5883L_REG_CONTROL2   0x0A
#define QMC5883L_REG_SET_RESET  0x0B
#define QMC5883L_REG_CHIP_ID    0x0D

/* ---- 罗盘数据结构 ---- */
typedef struct {
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    float   heading;        // 航向角 (度)
    int16_t temperature;
    uint8_t status;
} compass_data_t;

/* ---- 公开API ---- */
void compass_init(void);
uint8_t compass_read_data(compass_data_t *data);
float compass_get_heading(int16_t mag_x, int16_t mag_y, float declination);

#endif /* __SENSOR_COMPASS_H */
