/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : sensor_flow.c
 * 描述                : ATK-PMW3901 光流传感器 SPI 驱动实现
 *                       SPI3: PB3(SCK) PB4(MISO) PB5(MOSI) + PC5(CS)
 *                       轮询 SPI, ~2MHz, 100Hz 数据更新
 *
 * 算法参考: ATK-PMW3901 光流模块用户手册
 *******************************************************************************/
#include "sensor_flow.h"
#include "debug.h"
#include <math.h>
#include <stdlib.h>

/* ---- 硬件引脚 ---- */
#define FLOW_CS_PORT        GPIOC
#define FLOW_CS_PIN         GPIO_Pin_5

#define FLOW_SPI            SPI3
#define FLOW_SPI_CLK        RCC_HB1Periph_SPI3

#define FLOW_GPIO_AF        GPIO_AF6

/* CS 宏 */
#define CS_LOW()    GPIO_ResetBits(FLOW_CS_PORT, FLOW_CS_PIN)
#define CS_HIGH()   GPIO_SetBits(FLOW_CS_PORT, FLOW_CS_PIN)

/* ---- 内部状态 ---- */
static uint16_t g_error_count;     /* 连续 maxRaw/minRaw 为 0 的帧数 */

/* ---- 内部函数声明 ---- */
static uint8_t  spi3_transfer(uint8_t tx);
static uint8_t  registerRead(uint8_t reg);
static void     registerWrite(uint8_t reg, uint8_t val);
static void     readMotion(motionBurst_t *burst);
static void     InitRegisters(void);

/*********************************************************************
 * @fn      spi3_transfer
 * @brief   SPI3 单字节全双工传输 (轮询)
 */
static uint8_t spi3_transfer(uint8_t tx)
{
    while (SPI_I2S_GetFlagStatus(FLOW_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(FLOW_SPI, tx);
    while (SPI_I2S_GetFlagStatus(FLOW_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(FLOW_SPI);
}

/*********************************************************************
 * @fn      registerRead
 * @brief   读 PMW3901 寄存器
 *          CS 拉低 → 发送 (reg & 0x7F) → 发送 dummy → 接收数据 → CS 拉高
 */
static uint8_t registerRead(uint8_t reg)
{
    uint8_t val;
    CS_LOW();
    spi3_transfer(reg & 0x7F);
    val = spi3_transfer(0x00);
    CS_HIGH();
    return val;
}

/*********************************************************************
 * @fn      registerWrite
 * @brief   写 PMW3901 寄存器
 *          CS 拉低 → 发送 (reg | 0x80) → 发送 val → CS 拉高
 */
static void registerWrite(uint8_t reg, uint8_t val)
{
    CS_LOW();
    spi3_transfer(reg | 0x80);
    spi3_transfer(val);
    CS_HIGH();
}

/*********************************************************************
 * @fn      readMotion
 * @brief   突发读取 12 字节运动数据 (寄存器 0x16)
 */
static void readMotion(motionBurst_t *burst)
{
    uint8_t *p = (uint8_t *)burst;
    CS_LOW();
    spi3_transfer(PMW3901_REG_MOTION_BURST);
    for (uint8_t i = 0; i < 12; i++) {
        p[i] = spi3_transfer(0x00);
    }
    CS_HIGH();
}

/*********************************************************************
 * @fn      InitRegisters
 * @brief   PMW3901 推荐寄存器初始化序列
 */
static void InitRegisters(void)
{
    registerWrite(0x7F, 0x00);
    registerWrite(0x55, 0x01);
    registerWrite(0x50, 0x07);
    registerWrite(0x7F, 0x0E);
    registerWrite(0x43, 0x10);
    registerWrite(0x48, 0x04);
    registerWrite(0x7F, 0x0D);
    registerWrite(0x48, 0x18);
    registerWrite(0x52, 0x16);
    registerWrite(0x7F, 0x0A);
    registerWrite(0x5F, 0x32);
    registerWrite(0x61, 0x96);
    registerWrite(0x7F, 0x00);
    registerWrite(0x55, 0x02);
    registerWrite(0x7F, 0x00);
    registerWrite(0x60, 0x20);
    registerWrite(0x61, 0x00);
    registerWrite(0x7F, 0x14);
    registerWrite(0x44, 0xB0);
    registerWrite(0x5B, 0x04);
    registerWrite(0x5F, 0x44);
    registerWrite(0x6D, 0x01);
    registerWrite(0x7F, 0x15);
    registerWrite(0x4C, 0x04);
    registerWrite(0x7F, 0x00);
    registerWrite(0x40, 0x80);
}

/*********************************************************************
 * @fn      sensor_flow_init
 * @brief   初始化 SPI3 + PMW3901 寄存器
 *
 * 步骤:
 *   1. GPIO 时钟 + AF 配置 (PB3/4/5=SPI3, PC5=CS)
 *   2. 禁用 JTAG (PB3/4 复用)
 *   3. SPI3 初始化 (Mode3, ~2MHz, 8bit, 全双工主模式)
 *   4. PMW3901 上电复位 + 寄存器初始化
 *   5. 验证 Chip ID
 */
void sensor_flow_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    SPI_InitTypeDef   SPI_InitStructure  = {0};

    /* ---- 1. 时钟使能 ---- */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB | RCC_HB2Periph_GPIOC, ENABLE);
    RCC_HB1PeriphClockCmd(FLOW_SPI_CLK, ENABLE);

    /* ---- 2. PB3 = SPI3_SCK (AF6, AF_PP) ---- */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, FLOW_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ---- 3. PB4 = SPI3_MISO (AF6, IN_FLOATING) ---- */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, FLOW_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ---- 4. PB5 = SPI3_MOSI (AF6, AF_PP) ---- */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, FLOW_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ---- 5. PC5 = CS (OUT_PP, 初始高) ---- */
    GPIO_InitStructure.GPIO_Pin   = FLOW_CS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(FLOW_CS_PORT, &GPIO_InitStructure);
    CS_HIGH();

    /* ---- 6. SPI3 初始化 (Mode3, 全双工主, 8bit, ~2MHz) ---- */
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;       /* Mode3: CLK 空闲高 */
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;      /* Mode3: 第2边沿采样 */
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode6; /* 120MHz/128 ≈ 1.875MHz */
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(FLOW_SPI, &SPI_InitStructure);

    SPI_Cmd(FLOW_SPI, ENABLE);

    /* ---- 7. PMW3901 上电复位 ---- */
    CS_HIGH();
    Delay_Ms(50);
    registerWrite(PMW3901_REG_POWER_UP_RESET, 0x5A);
    Delay_Ms(5);

    /* ---- 8. 寄存器初始化 ---- */
    InitRegisters();
    Delay_Ms(5);

    /* ---- 9. 验证 Chip ID ---- */
    uint8_t chipId    = registerRead(PMW3901_REG_PRODUCT_ID);
    uint8_t invChipId = registerRead(PMW3901_REG_INV_PRODUCT_ID);
    printf("[FLOW] PMW3901 Product_ID=0x%02X Inv_ID=0x%02X\r\n", chipId, invChipId);

    if (chipId == 0x49 && invChipId == 0xB6) {
        printf("[FLOW] PMW3901 初始化成功\r\n");
    } else {
        printf("[FLOW] PMW3901 ID 异常, 请检查接线: PB3/SCK PB4/MISO PB5/MOSI PC5/CS\r\n");
    }

    g_error_count = 0;
}

/*********************************************************************
 * @fn      sensor_flow_poll
 * @brief   读取并处理一帧光流数据 (100Hz 调用)
 *
 * @param   flow:      光流状态结构体指针
 * @param   height_m:  当前高度 (m), 来自 TFmini TOF
 * @param   roll_rad:  横滚角 (rad)
 * @param   pitch_rad: 俯仰角 (rad)
 * @param   dt:        距离上次调用的时间间隔 (s)
 *
 * 算法:
 *   1. 突发读取 12 字节运动数据
 *   2. 检测传感器故障 (maxRaw/minRaw 连续 1s 为 0)
 *   3. 像素方向适配 (pitch=x, roll=y), 取反
 *   4. 野值限幅
 *   5. 倾角补偿
 *   6. 像素 → 位移 (cm)
 *   7. 位移 → 速度 (cm/s), 低通 + 限幅
 */
void sensor_flow_poll(opFlow_t *flow, float height_m,
                      float roll_rad, float pitch_rad, float dt)
{
    motionBurst_t burst;
    readMotion(&burst);

    /* 传感器故障检测: maxRaw/minRaw 连续为 0 超过 1s */
    if (burst.maxRawData == 0 && burst.minRawData == 0) {
        if (++g_error_count > 100) {
            g_error_count = 0;
            flow->isOpFlowOk = false;
        }
        return;
    }
    g_error_count = 0;
    flow->isOpFlowOk = true;

    /* 像素方向适配: MiniFly pitch=x, roll=y, 取反 */
    int16_t pixelDx =  burst.deltaY;
    int16_t pixelDy = -burst.deltaX;

    /* 野值限幅 */
    if (abs(pixelDx) < OUTLIER_LIMIT && abs(pixelDy) < OUTLIER_LIMIT) {
        flow->pixSum[0] += pixelDx;
        flow->pixSum[1] += pixelDy;
    }

    /* 倾角补偿 */
    flow->pixComp[0] = COMP_COEFF * tanf(pitch_rad);
    flow->pixComp[1] = COMP_COEFF * tanf(roll_rad);

    /* 有效像素 = 累积像素 + 倾角补偿 */
    flow->pixValid[0] = flow->pixSum[0] + flow->pixComp[0];
    flow->pixValid[1] = flow->pixSum[1] + flow->pixComp[1];

    /* 像素 → 位移系数 (高度相关) */
    float coeff = RESOLUTION * height_m;
    if (height_m < 0.05f) {
        coeff = 0.0f;  /* < 5cm 光流不可用 */
    }

    /* 帧间位移 (cm) */
    flow->deltaPos[0] = coeff * (flow->pixValid[0] - flow->pixValidLast[0]);
    flow->deltaPos[1] = coeff * (flow->pixValid[1] - flow->pixValidLast[1]);

    flow->pixValidLast[0] = flow->pixValid[0];
    flow->pixValidLast[1] = flow->pixValid[1];

    /* 瞬时速度 (cm/s) */
    if (dt > 0.001f) {
        flow->deltaVel[0] = flow->deltaPos[0] / dt;
        flow->deltaVel[1] = flow->deltaPos[1] / dt;
    }

    /* 速度低通滤波 */
    flow->velLpf[0] += (flow->deltaVel[0] - flow->velLpf[0]) * 0.15f;
    flow->velLpf[1] += (flow->deltaVel[1] - flow->velLpf[1]) * 0.15f;

    /* 速度限幅 */
    if (flow->velLpf[0] >  VEL_LIMIT) flow->velLpf[0] =  VEL_LIMIT;
    if (flow->velLpf[0] < -VEL_LIMIT) flow->velLpf[0] = -VEL_LIMIT;
    if (flow->velLpf[1] >  VEL_LIMIT) flow->velLpf[1] =  VEL_LIMIT;
    if (flow->velLpf[1] < -VEL_LIMIT) flow->velLpf[1] = -VEL_LIMIT;

    /* 累积位移 */
    flow->posSum[0] += flow->deltaPos[0];
    flow->posSum[1] += flow->deltaPos[1];

    /* 数据有效性: 高度 4m 以内 */
    flow->isDataValid = (height_m >= 0.05f && height_m < 4.0f);
}

/*********************************************************************
 * @fn      sensor_flow_reset
 * @brief   重置光流累积数据 (起飞/模式切换时调用)
 */
void sensor_flow_reset(opFlow_t *flow)
{
    flow->pixSum[0]       = 0.0f;
    flow->pixSum[1]       = 0.0f;
    flow->pixComp[0]      = 0.0f;
    flow->pixComp[1]      = 0.0f;
    flow->pixValid[0]     = 0.0f;
    flow->pixValid[1]     = 0.0f;
    flow->pixValidLast[0] = 0.0f;
    flow->pixValidLast[1] = 0.0f;
    flow->deltaPos[0]     = 0.0f;
    flow->deltaPos[1]     = 0.0f;
    flow->deltaVel[0]     = 0.0f;
    flow->deltaVel[1]     = 0.0f;
    flow->posSum[0]       = 0.0f;
    flow->posSum[1]       = 0.0f;
    flow->velLpf[0]       = 0.0f;
    flow->velLpf[1]       = 0.0f;
    flow->isOpFlowOk      = true;
    flow->isDataValid     = false;
    g_error_count          = 0;
}
