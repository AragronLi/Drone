/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : sensor_compass.c
 * 描述                : QMC5883L 罗盘传感器 I2C 驱动实现
 *                       I2C3: PA14(SCL) + PA13(SDA), AF7, 400kHz
 *                       SR2525M10D 集成 QMC5883L 罗盘
 *******************************************************************************/
#include "sensor_compass.h"
#include "debug.h"
#include <math.h>

#define COMPASS_I2C              I2C3
#define COMPASS_I2C_TIMEOUT      200000UL

static uint8_t g_compass_i2c_error;

static uint8_t compass_wait_event(uint32_t event)
{
    uint32_t timeout = COMPASS_I2C_TIMEOUT;

    while (I2C_CheckEvent(COMPASS_I2C, event) != SUCCESS) {
        if (I2C_GetFlagStatus(COMPASS_I2C, I2C_FLAG_AF) != RESET) {
            I2C_ClearFlag(COMPASS_I2C, I2C_FLAG_AF);
            I2C_GenerateSTOP(COMPASS_I2C, ENABLE);
            g_compass_i2c_error = 1;
            return 0;
        }
        if (timeout-- == 0) {
            I2C_GenerateSTOP(COMPASS_I2C, ENABLE);
            g_compass_i2c_error = 1;
            return 0;
        }
    }

    return 1;
}

static uint8_t compass_wait_bus_free(void)
{
    uint32_t timeout = COMPASS_I2C_TIMEOUT;

    while (I2C_GetFlagStatus(COMPASS_I2C, I2C_FLAG_BUSY) != RESET) {
        if (timeout-- == 0) {
            g_compass_i2c_error = 1;
            return 0;
        }
    }

    return 1;
}

static void compass_i2c3_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitStructure  = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA | RCC_HB2Periph_AFIO, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_I2C3, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF7);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF7);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    I2C_InitStructure.I2C_ClockSpeed          = 400000;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(COMPASS_I2C, &I2C_InitStructure);
    I2C_Cmd(COMPASS_I2C, ENABLE);
}

/* ---- I2C写寄存器 ---- */
static uint8_t compass_write_reg(uint8_t reg, uint8_t val)
{
    if (!compass_wait_bus_free()) return 0;

    I2C_GenerateSTART(COMPASS_I2C, ENABLE);
    if (!compass_wait_event(I2C_EVENT_MASTER_MODE_SELECT)) return 0;

    I2C_Send7bitAddress(COMPASS_I2C, QMC5883L_ADDR, I2C_Direction_Transmitter);
    if (!compass_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0;

    I2C_SendData(COMPASS_I2C, reg);
    if (!compass_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0;

    I2C_SendData(COMPASS_I2C, val);
    if (!compass_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0;

    I2C_GenerateSTOP(COMPASS_I2C, ENABLE);
    return 1;
}

/* ---- I2C读寄存器 ---- */
static uint8_t compass_read_reg(uint8_t reg, uint8_t *val)
{
    if (!compass_wait_bus_free()) return 0;

    I2C_GenerateSTART(COMPASS_I2C, ENABLE);
    if (!compass_wait_event(I2C_EVENT_MASTER_MODE_SELECT)) return 0;

    I2C_Send7bitAddress(COMPASS_I2C, QMC5883L_ADDR, I2C_Direction_Transmitter);
    if (!compass_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0;

    I2C_SendData(COMPASS_I2C, reg);
    if (!compass_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0;

    I2C_GenerateSTART(COMPASS_I2C, ENABLE);
    if (!compass_wait_event(I2C_EVENT_MASTER_MODE_SELECT)) return 0;

    I2C_Send7bitAddress(COMPASS_I2C, QMC5883L_ADDR, I2C_Direction_Receiver);
    if (!compass_wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return 0;

    I2C_AcknowledgeConfig(COMPASS_I2C, DISABLE);

    if (!compass_wait_event(I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        I2C_AcknowledgeConfig(COMPASS_I2C, ENABLE);
        return 0;
    }
    *val = I2C_ReceiveData(COMPASS_I2C);

    I2C_GenerateSTOP(COMPASS_I2C, ENABLE);
    I2C_AcknowledgeConfig(COMPASS_I2C, ENABLE);

    return 1;
}

/* ---- 初始化罗盘 ---- */
void compass_init(void)
{
    uint8_t chip_id = 0;

    g_compass_i2c_error = 0;
    compass_i2c3_init();
    Delay_Ms(10);

    /* SET/RESET 周期 */
    if (!compass_write_reg(QMC5883L_REG_SET_RESET, 0x01)) {
        printf("QMC5883L SET/RESET write failed, check I2C3 PA14/PA13 and address 0x%02X\r\n", QMC5883L_ADDR);
        return;
    }
    Delay_Ms(10);

    /* 配置: 200Hz ODR, ±2G, 512 OSR, 连续模式 */
    if (!compass_write_reg(QMC5883L_REG_CONTROL1, 0x1D)) {
        printf("QMC5883L CONTROL1 write failed, check I2C3 PA14/PA13 and address 0x%02X\r\n", QMC5883L_ADDR);
        return;
    }
    Delay_Ms(10);

    if (compass_read_reg(QMC5883L_REG_CHIP_ID, &chip_id)) {
        printf("QMC5883L Chip ID: 0x%02X (expected 0xFF)\r\n", chip_id);
    } else {
        printf("QMC5883L Chip ID read failed, check I2C3 PA14/PA13 and address 0x%02X\r\n", QMC5883L_ADDR);
    }
}

/* ---- 读取罗盘数据 ---- */
uint8_t compass_read_data(compass_data_t *data)
{
    uint8_t buf[6];

    for (uint8_t i = 0; i < 6; i++) {
        if (!compass_read_reg(QMC5883L_REG_DATA_X_L + i, &buf[i])) {
            data->status = 0;
            return 0;
        }
    }

    data->mag_x = (int16_t)(buf[1] << 8 | buf[0]);
    data->mag_y = (int16_t)(buf[3] << 8 | buf[2]);
    data->mag_z = (int16_t)(buf[5] << 8 | buf[4]);

    if (!compass_read_reg(QMC5883L_REG_STATUS, &data->status)) {
        data->status = 0;
        return 0;
    }

    return (data->status & 0x01);
}

/* ---- 计算航向角 ---- */
float compass_get_heading(int16_t mag_x, int16_t mag_y, float declination)
{
    float heading = atan2f((float)mag_y, (float)mag_x) * 180.0f / 3.14159265f;
    heading += declination;

    if (heading < 0) heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;

    return heading;
}
