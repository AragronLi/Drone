/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : oled_ssd1306.h
 * 描述                : SSD1306 OLED 128x64 I2C 驱动头文件
 *                       I2C3: PA14(SCL) + PA13(SDA), AF7, 400kHz
 *                       器件地址: 0x3C (与罗盘QMC5883L共享I2C3总线)
 *******************************************************************************/

#ifndef __OLED_SSD1306_H
#define __OLED_SSD1306_H

#include "ch32h417.h"

/* ---- 显示尺寸 ---- */
#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)

/* ---- I2C 地址 ---- */
#define OLED_I2C_ADDR  0x3C

/* ---- I2C 控制字节 ---- */
#define OLED_CTRL_CMD  0x00   /* 下一字节为命令 */
#define OLED_CTRL_DATA 0x40   /* 下一字节为数据 */

/* ---- 字体尺寸 ---- */
#define FONT_6X8_WIDTH   6
#define FONT_6X8_HEIGHT  8
#define FONT_8X16_WIDTH  8
#define FONT_8X16_HEIGHT 16

/* ---- 公开 API ---- */
void oled_init(void);
void oled_clear(void);
void oled_refresh(void);
void oled_clear_region(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void oled_show_char(uint8_t x, uint8_t y, char ch, uint8_t size);
void oled_show_string(uint8_t x, uint8_t y, const char *str, uint8_t size);
void oled_show_num(uint8_t x, uint8_t y, float num, uint8_t decimals, uint8_t size);
void oled_show_int(uint8_t x, uint8_t y, int32_t num, uint8_t size);

#endif /* __OLED_SSD1306_H */
