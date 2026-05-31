/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : oled_ssd1306.c
 * 描述                : SSD1306 OLED 128x64 I2C 驱动实现
 *                       I2C3: PA14(SCL) + PA13(SDA), AF7, 400kHz Fast Mode
 *                       与罗盘QMC5883L (0x0D) 共享I2C3总线
 *                       帧缓冲 + 6x8 ASCII 字体 + 基本绘图 API
 *******************************************************************************/

#include "oled_ssd1306.h"
#include <string.h>
#include <stdio.h>

/* ============================================================
 *  6x8 ASCII 字体表 (0x20 ~ 0x7E, 95 字符)
 *  每字符 6 字节, 纵向取模, 高位在上
 * ============================================================ */
static const uint8_t g_font_6x8[95][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /*   */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* = */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* E */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* F */
    {0x3E,0x41,0x41,0x51,0x32,0x00}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* X */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* Z */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* \ */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* f */
    {0x08,0x14,0x54,0x54,0x3C,0x00}, /* g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* j */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* } */
    {0x08,0x04,0x08,0x10,0x08,0x00}, /* ~ */
};

/* ============================================================
 *  帧缓冲区: 128 × 64 / 8 = 1024 字节
 *  组织方式: 8 页 (Page0~7) × 128 列 (Col0~127)
 *  每字节 bit0 对应页顶部像素, bit7 对应底部
 * ============================================================ */
static uint8_t g_oled_fb[OLED_WIDTH * OLED_PAGES];

static int32_t pow10(uint8_t n);

/* ============================================================
 *  I2C3 底层操作
 * ============================================================ */

#define OLED_I2C  I2C3

static void i2c_start(void)
{
    while (I2C_GetFlagStatus(OLED_I2C, I2C_FLAG_BUSY));
    I2C_GenerateSTART(OLED_I2C, ENABLE);
    while (I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
}

static void i2c_stop(void)
{
    I2C_GenerateSTOP(OLED_I2C, ENABLE);
}

static void i2c_send_addr(uint8_t addr, uint8_t dir)
{
    I2C_Send7bitAddress(OLED_I2C, addr, dir);
    if (dir == I2C_Direction_Transmitter) {
        while (I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);
    }
}

static void i2c_send_byte(uint8_t dat)
{
    I2C_SendData(OLED_I2C, dat);
    while (I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
}

/*********************************************************************
 * @fn      oled_write_cmd
 * @brief   向 SSD1306 写入一条命令
 */
static void oled_write_cmd(uint8_t cmd)
{
    i2c_start();
    i2c_send_addr(OLED_I2C_ADDR, I2C_Direction_Transmitter);
    i2c_send_byte(OLED_CTRL_CMD);
    i2c_send_byte(cmd);
    i2c_stop();
}

/*********************************************************************
 * @fn      oled_write_data_burst
 * @brief   向 SSD1306 突发写入数据块
 */
static void oled_write_data_burst(const uint8_t *dat, uint16_t len)
{
    i2c_start();
    i2c_send_addr(OLED_I2C_ADDR, I2C_Direction_Transmitter);
    i2c_send_byte(OLED_CTRL_DATA);
    for (uint16_t i = 0; i < len; i++) {
        i2c_send_byte(dat[i]);
    }
    i2c_stop();
}

/* ============================================================
 *  SSD1306 初始化序列
 * ============================================================ */

static void oled_init_sequence(void)
{
    oled_write_cmd(0xAE);  /* 关闭显示 */

    oled_write_cmd(0xD5);  /* 时钟分频 / 振荡器频率 */
    oled_write_cmd(0x80);

    oled_write_cmd(0xA8);  /* 复用率 */
    oled_write_cmd(0x3F);  /* 64 行 */

    oled_write_cmd(0xD3);  /* 显示偏移 */
    oled_write_cmd(0x00);

    oled_write_cmd(0x40);  /* 起始行 */

    oled_write_cmd(0x8D);  /* 充电泵 */
    oled_write_cmd(0x14);  /* 使能 */

    oled_write_cmd(0x20);  /* 内存寻址模式 */
    oled_write_cmd(0x00);  /* 水平寻址 */

    oled_write_cmd(0xA1);  /* 段重映射 (col127→col0) */
    oled_write_cmd(0xC8);  /* COM 扫描方向 (COM63→COM0) */

    oled_write_cmd(0xDA);  /* COM 引脚硬件配置 */
    oled_write_cmd(0x12);

    oled_write_cmd(0x81);  /* 对比度 */
    oled_write_cmd(0xCF);

    oled_write_cmd(0xD9);  /* 预充电周期 */
    oled_write_cmd(0xF1);

    oled_write_cmd(0xDB);  /* VCOMH 取消选择电平 */
    oled_write_cmd(0x40);

    oled_write_cmd(0xA4);  /* 全屏点亮关闭 → 跟随 GDDRAM */
    oled_write_cmd(0xA6);  /* 正常显示 (非反色) */

    oled_write_cmd(0x2E);  /* 停止滚动 */

    oled_write_cmd(0xAF);  /* 开启显示 */
}

/* ============================================================
 *  公开 API
 * ============================================================ */

/*********************************************************************
 * @fn      oled_init
 * @brief   初始化 I2C3 (PA14/PA13, AF7, 400kHz) + SSD1306 初始化序列
 */
void oled_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitStructure  = {0};

    /* ---- 1. 时钟使能 ---- */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA | RCC_HB2Periph_AFIO, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_I2C3, ENABLE);

    /* ---- 2. PA14 = I2C3_SCL (AF7, 开漏) ---- */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF7);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ---- 3. PA13 = I2C3_SDA (AF7, 开漏) ---- */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF7);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ---- 4. I2C3 参数配置 (400kHz Fast Mode) ---- */
    I2C_InitStructure.I2C_ClockSpeed          = 400000;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;  /* 主机模式, 自身地址无关 */
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(OLED_I2C, &I2C_InitStructure);

    I2C_Cmd(OLED_I2C, ENABLE);

    /* ---- 5. SSD1306 初始化序列 ---- */
    oled_init_sequence();

    /* ---- 6. 清空帧缓冲 + 刷新 ---- */
    oled_clear();
    oled_refresh();
}

/*********************************************************************
 * @fn      oled_clear
 * @brief   清空帧缓冲区 (不刷新屏幕, 需调用 oled_refresh)
 */
void oled_clear(void)
{
    memset(g_oled_fb, 0x00, sizeof(g_oled_fb));
}

/*********************************************************************
 * @fn      oled_clear_region
 * @brief   清空帧缓冲区指定区域
 */
void oled_clear_region(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    uint8_t page_start = y / 8;
    uint8_t page_end   = (y + h - 1) / 8;
    uint8_t bit_mask   = 0xFF;

    if (page_start == page_end) {
        /* 单页内部分清空 */
        uint8_t mask = ~(((1 << h) - 1) << (y % 8));
        for (uint8_t col = x; col < x + w && col < OLED_WIDTH; col++) {
            g_oled_fb[page_start * OLED_WIDTH + col] &= mask;
        }
        return;
    }

    for (uint8_t p = page_start; p <= page_end && p < OLED_PAGES; p++) {
        if (p == page_start) {
            bit_mask = ~(0xFF << (y % 8));
        } else if (p == page_end) {
            bit_mask = ~((1 << ((y + h) % 8)) - 1);
        } else {
            bit_mask = 0x00;
        }
        for (uint8_t col = x; col < x + w && col < OLED_WIDTH; col++) {
            g_oled_fb[p * OLED_WIDTH + col] &= bit_mask;
        }
    }
}

/*********************************************************************
 * @fn      oled_refresh
 * @brief   将帧缓冲区全量写入 SSD1306 GDDRAM (I2C 突发)
 */
void oled_refresh(void)
{
    /* 设置 GDDRAM 写入范围: 页 0~7, 列 0~127 */
    oled_write_cmd(0x21);  /* 列地址范围 */
    oled_write_cmd(0x00);
    oled_write_cmd(127);
    oled_write_cmd(0x22);  /* 页地址范围 */
    oled_write_cmd(0x00);
    oled_write_cmd(7);

    /* 突发写入全部 1024 字节 */
    oled_write_data_burst(g_oled_fb, sizeof(g_oled_fb));
}

/*********************************************************************
 * @fn      oled_show_char
 * @brief   在帧缓冲区绘制一个字符
 * @param   x, y: 左上角坐标 (像素)
 * @param   ch   : ASCII 字符
 * @param   size : FONT_6X8_WIDTH/HEIGHT (仅支持 6x8)
 */
void oled_show_char(uint8_t x, uint8_t y, char ch, uint8_t size)
{
    if (x > OLED_WIDTH - 6 || y > OLED_HEIGHT - 8) return;
    if (ch < 0x20 || ch > 0x7E) ch = ' ';

    uint8_t idx = ch - 0x20;
    uint8_t page = y / 8;
    uint8_t y_off = y % 8;

    for (uint8_t col = 0; col < 6; col++) {
        uint16_t fb_addr = page * OLED_WIDTH + x + col;
        uint8_t  font_byte = g_font_6x8[idx][col];

        if (y_off == 0) {
            g_oled_fb[fb_addr] &= 0x00;
            g_oled_fb[fb_addr] |= font_byte;
        } else {
            /* 跨页处理 */
            g_oled_fb[fb_addr] &= (0xFF >> (8 - y_off));
            g_oled_fb[fb_addr] |= (font_byte << y_off);
            if (page + 1 < OLED_PAGES) {
                g_oled_fb[fb_addr + OLED_WIDTH] &= (0xFF << y_off);
                g_oled_fb[fb_addr + OLED_WIDTH] |= (font_byte >> (8 - y_off));
            }
        }
    }
}

/*********************************************************************
 * @fn      oled_show_string
 * @brief   在帧缓冲区绘制字符串
 */
void oled_show_string(uint8_t x, uint8_t y, const char *str, uint8_t size)
{
    while (*str) {
        oled_show_char(x, y, *str, size);
        x += FONT_6X8_WIDTH;
        if (x > OLED_WIDTH - 6) {
            x = 0;
            y += FONT_6X8_HEIGHT;
        }
        if (y > OLED_HEIGHT - 8) break;
        str++;
    }
}

/*********************************************************************
 * @fn      oled_show_num
 * @brief   在帧缓冲区绘制浮点数
 */
void oled_show_num(uint8_t x, uint8_t y, float num, uint8_t decimals, uint8_t size)
{
    char buf[16];
    if (decimals == 0) {
        snprintf(buf, sizeof(buf), "%ld", (long)((int32_t)num));
    } else {
        /* 简易浮点格式化 */
        int32_t int_part = (int32_t)num;
        int32_t dec_part = (int32_t)((num - int_part) * (int32_t)pow10(decimals));
        if (dec_part < 0) dec_part = -dec_part;
        snprintf(buf, sizeof(buf), "%ld.%0*ld", (long)int_part, decimals, (long)dec_part);
    }
    oled_show_string(x, y, buf, size);
}

/*********************************************************************
 * @fn      oled_show_int
 * @brief   在帧缓冲区绘制整数
 */
void oled_show_int(uint8_t x, uint8_t y, int32_t num, uint8_t size)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%ld", (long)num);
    oled_show_string(x, y, buf, size);
}

/*********************************************************************
 * @fn      pow10
 * @brief   返回 10^n (内部辅助函数)
 */
static int32_t pow10(uint8_t n)
{
    int32_t v = 1;
    while (n--) v *= 10;
    return v;
}
