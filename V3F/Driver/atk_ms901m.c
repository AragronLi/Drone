/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : atk_ms901m.c
 * 作者                : WCH
 * 版本                : V1.0.0
 * 日期                : 2025/05/28
 * 描述                : ATK-MS901M 十轴 IMU 协议解析器
 *                      帧格式（二进制，小端序）:
 *                        [0]     帧头低位   = 0x55
 *                        [1]     帧头高位   = 0x55 (上传) / 0xAF (应答)
 *                        [2]     帧 ID
 *                        [3]     数据长度
 *                        [4..N]  数据字段
 *                        [N+1]   校验和（前 N+1 字节累加和的低 8 位）
 *
 *                      传感器: ICM-20602 + AK8963C + SPL06-001
 *                      参考: 正点原子 ATK-MS901M 模块用户手册 V1.0
 *********************************************************************************
 * Copyright (c) 2025 南京沁恒微电子股份有限公司
 *******************************************************************************/

#include "atk_ms901m.h"
#include "atk_ms901m_uart.h"
#include "debug.h"

/* ---- 帧头字节 ---- */
#define FRAME_HEAD_L                  0x55   /* 帧头低位 */
#define FRAME_HEAD_UPLOAD_H           0x55   /* 上传帧头高位 */
#define FRAME_HEAD_ACK_H              0xAF   /* 应答帧头高位 */

/* 寄存器读写 ID 标志位 */
#define READ_REG_ID(id)               ((id) | 0x80)
#define WRITE_REG_ID(id)              (id)

/* ---- 通信帧结构 ---- */
typedef struct {
    uint8_t head_l;                              /* 帧头低位 */
    uint8_t head_h;                              /* 帧头高位 */
    uint8_t id;                                  /* 帧 ID */
    uint8_t len;                                 /* 数据长度 */
    uint8_t dat[ATK_MS901M_FRAME_DAT_MAX_SIZE];  /* 数据字段 */
    uint8_t check_sum;                           /* 校验和 */
} atk_ms901m_frame_t;

/* ---- 帧解析状态机 ---- */
typedef enum {
    WAIT_HEAD_L  = 0x00,   /* 等待帧头低位 0x55 */
    WAIT_HEAD_H  = 0x01,   /* 等待帧头高位 */
    WAIT_ID      = 0x02,   /* 等待帧 ID */
    WAIT_LEN     = 0x04,   /* 等待数据长度 */
    WAIT_DAT     = 0x08,   /* 等待数据字段 */
    WAIT_SUM     = 0x10,   /* 等待校验和 */
} atk_ms901m_handle_state_t;

/* ---- 量程查找表 ---- */
static const uint16_t g_gyro_fsr_table[4]     = {250, 500, 1000, 2000};  /* 陀螺仪量程: dps */
static const uint8_t  g_accel_fsr_table[4]    = {2, 4, 8, 16};           /* 加速度计量程: G */

/* ---- 缓存的量程设置（初始化时从模块读取） ---- */
static uint8_t g_gyro_fsr_idx  = 1;  /* 默认 500 dps */
static uint8_t g_accel_fsr_idx = 1;  /* 默认 4 G */

/*********************************************************************
 * @fn      atk_ms901m_get_frame_by_id
 * @brief   状态机帧解析器：从 FIFO 逐字节读取，直到匹配到指定 ID
 *          且校验和正确的完整帧
 * @param   frame   : 输出的解析后帧
 *          id      : 期望的帧 ID
 *          id_type : 帧类型（上传帧 / 应答帧）
 *          timeout : 最大等待时间，单位 ms（近似）
 * @retval  ATK_MS901M_EOK      : 成功接收匹配帧
 *          ATK_MS901M_EINVAL   : 参数无效
 *          ATK_MS901M_ETIMEOUT : 超时
 */
static uint8_t atk_ms901m_get_frame_by_id(atk_ms901m_frame_t *frame, uint8_t id,
                                          uint8_t id_type, uint32_t timeout)
{
    uint8_t  dat;
    atk_ms901m_handle_state_t state = WAIT_HEAD_L;
    uint8_t  dat_index = 0;
    uint16_t tick = 0;

    while (1) {
        if (timeout == 0) {
            return ATK_MS901M_ETIMEOUT;
        }

        /* FIFO 无数据则等待 100us 后重试 */
        if (atk_ms901m_uart_rx_fifo_read(&dat, 1) == 0) {
            Delay_Us(100);
            tick++;
            if (tick >= 10) {
                tick = 0;
                timeout--;
            }
            continue;
        }

        switch (state) {
        case WAIT_HEAD_L:                            /* 等待帧头低位 0x55 */
            if (dat == FRAME_HEAD_L) {
                frame->head_l = dat;
                frame->check_sum = dat;
                state = WAIT_HEAD_H;
            }
            break;

        case WAIT_HEAD_H:                            /* 等待帧头高位 */
            if (id_type == ATK_MS901M_FRAME_ID_TYPE_UPLOAD &&
                dat == FRAME_HEAD_UPLOAD_H) {
                frame->head_h = dat;
                frame->check_sum += dat;
                state = WAIT_ID;
            } else if (id_type == ATK_MS901M_FRAME_ID_TYPE_ACK &&
                       dat == FRAME_HEAD_ACK_H) {
                frame->head_h = dat;
                frame->check_sum += dat;
                state = WAIT_ID;
            } else {
                state = WAIT_HEAD_L;
            }
            break;

        case WAIT_ID:                                /* 等待帧 ID */
            if (dat == id) {
                frame->id = dat;
                frame->check_sum += dat;
                state = WAIT_LEN;
            } else {
                state = WAIT_HEAD_L;
            }
            break;

        case WAIT_LEN:                               /* 等待数据长度 */
            if (dat > ATK_MS901M_FRAME_DAT_MAX_SIZE) {
                state = WAIT_HEAD_L;                 /* 长度非法则重新同步 */
            } else {
                frame->len = dat;
                frame->check_sum += dat;
                state = (frame->len == 0) ? WAIT_SUM : WAIT_DAT;
            }
            break;

        case WAIT_DAT:                               /* 等待数据字段 */
            frame->dat[dat_index] = dat;
            frame->check_sum += dat;
            dat_index++;
            if (dat_index == frame->len) {
                dat_index = 0;
                state = WAIT_SUM;
            }
            break;

        case WAIT_SUM:                               /* 等待校验和 */
            if (dat == frame->check_sum) {
                return ATK_MS901M_EOK;
            }
            state = WAIT_HEAD_L;                     /* 校验失败则重新同步 */
            break;

        default:
            state = WAIT_HEAD_L;
            break;
        }

        Delay_Us(100);
        tick++;
        if (tick >= 10) {
            tick = 0;
            timeout--;
        }
    }
}

/*********************************************************************
 * @fn      atk_ms901m_read_reg_by_id
 * @brief   通过帧 ID 读取 IMU 模块寄存器
 * @param   id     : 寄存器对应的帧 ID
 *          dat    : 输出数据缓冲区
 *          timeout: 超时时间 ms
 * @retval  读取到的数据长度，失败返回 0
 */
static uint8_t atk_ms901m_read_reg_by_id(uint8_t id, uint8_t *dat, uint32_t timeout)
{
    uint8_t buf[6];
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    uint8_t i;

    /* 发送读命令: 0x55 0xAF id|0x80 0x01 0x00 checksum */
    buf[0] = FRAME_HEAD_L;
    buf[1] = FRAME_HEAD_ACK_H;
    buf[2] = READ_REG_ID(id);
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    atk_ms901m_uart_send(buf, 6);

    ret = atk_ms901m_get_frame_by_id(&frame, id, ATK_MS901M_FRAME_ID_TYPE_ACK, timeout);
    if (ret != ATK_MS901M_EOK) {
        return 0;
    }

    for (i = 0; i < frame.len; i++) {
        dat[i] = frame.dat[i];
    }
    return frame.len;
}

/*********************************************************************
 * @fn      atk_ms901m_write_reg_by_id
 * @brief   通过帧 ID 写入 IMU 模块寄存器
 * @param   id : 寄存器对应的帧 ID
 *          len: 写入数据长度（1 或 2 字节）
 *          dat: 待写入的数据
 * @retval  ATK_MS901M_EOK / ATK_MS901M_EINVAL
 */
static uint8_t atk_ms901m_write_reg_by_id(uint8_t id, uint8_t len, uint8_t *dat)
{
    uint8_t buf[7];

    buf[0] = FRAME_HEAD_L;
    buf[1] = FRAME_HEAD_ACK_H;
    buf[2] = WRITE_REG_ID(id);
    buf[3] = len;

    if (len == 1) {
        buf[4] = dat[0];
        buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
        atk_ms901m_uart_send(buf, 6);
    } else if (len == 2) {
        buf[4] = dat[0];
        buf[5] = dat[1];
        buf[6] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5];
        atk_ms901m_uart_send(buf, 7);
    } else {
        return ATK_MS901M_EINVAL;
    }
    return ATK_MS901M_EOK;
}

/* ======================== 公开接口 ======================== */

/*********************************************************************
 * @fn      atk_ms901m_init
 * @brief   初始化 IMU 模块: UART 初始化 → 读取量程配置 →
 *          设置回传内容与频率 → 清空 FIFO
 * @param   baudrate: 串口波特率（须与模块当前设置一致）
 * @retval  ATK_MS901M_EOK 成功
 */
uint8_t atk_ms901m_init(uint32_t baudrate)
{
    uint8_t ret;
    uint8_t val;

    atk_ms901m_uart_init(baudrate);
    Delay_Ms(100);

    /* 读取陀螺仪量程 */
    ret = atk_ms901m_read_reg_by_id(ATK_MS901M_REG_ID_GYROFSR, &g_gyro_fsr_idx, 200);
    if (ret == 0) {
        printf("[IMU] 警告: 读取陀螺仪量程失败，使用默认值\r\n");
    } else {
        printf("[IMU] 陀螺仪量程: %d dps\r\n", g_gyro_fsr_table[g_gyro_fsr_idx & 0x03]);
    }

    /* 读取加速度计量程 */
    ret = atk_ms901m_read_reg_by_id(ATK_MS901M_REG_ID_ACCFSR, &g_accel_fsr_idx, 200);
    if (ret == 0) {
        printf("[IMU] 警告: 读取加速度计量程失败，使用默认值\r\n");
    } else {
        printf("[IMU] 加速度计量程: %d G\r\n", g_accel_fsr_table[g_accel_fsr_idx & 0x03]);
    }

    /* 配置回传内容: 姿态 + 陀螺仪/加速度计 */
    val = (1 << 0) | (1 << 2);
    atk_ms901m_write_reg_by_id(ATK_MS901M_REG_ID_RETURNSET, 1, &val);

    /* 配置回传频率: 200Hz (索引值 10) */
    val = 10;
    atk_ms901m_write_reg_by_id(ATK_MS901M_REG_ID_RETURNRATE, 1, &val);

    /* 清空残留数据 */
    Delay_Ms(50);
    atk_ms901m_uart_rx_fifo_flush();

    printf("[IMU] 初始化完成, 波特率=%d\r\n", (int)baudrate);
    return ATK_MS901M_EOK;
}

/*********************************************************************
 * @fn      atk_ms901m_get_attitude
 * @brief   等待并解析姿态上传帧 (ID 0x01)
 *          数据: 横滚(2B) + 俯仰(2B) + 偏航(2B)
 *          转换: int16 / 32768 * 180°
 */
uint8_t atk_ms901m_get_attitude(atk_ms901m_attitude_data_t *dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (dat == NULL) return ATK_MS901M_ERROR;

    ret = atk_ms901m_get_frame_by_id(&frame, ATK_MS901M_FRAME_ID_ATTITUDE,
                                     ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) return ATK_MS901M_ERROR;

    dat->roll  = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768.0f * 180.0f;
    dat->pitch = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768.0f * 180.0f;
    dat->yaw   = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768.0f * 180.0f;

    return ATK_MS901M_EOK;
}

/*********************************************************************
 * @fn      atk_ms901m_get_quaternion
 * @brief   等待并解析四元数上传帧 (ID 0x02)
 *          数据: q0~q3, 各 2 字节
 *          转换: int16 / 32768
 */
uint8_t atk_ms901m_get_quaternion(atk_ms901m_quaternion_data_t *dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (dat == NULL) return ATK_MS901M_ERROR;

    ret = atk_ms901m_get_frame_by_id(&frame, ATK_MS901M_FRAME_ID_QUAT,
                                     ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) return ATK_MS901M_ERROR;

    dat->q0 = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768.0f;
    dat->q1 = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768.0f;
    dat->q2 = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768.0f;
    dat->q3 = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 32768.0f;

    return ATK_MS901M_EOK;
}

/*********************************************************************
 * @fn      atk_ms901m_get_gyro_accelerometer
 * @brief   等待并解析陀螺仪+加速度计上传帧 (ID 0x03)
 *          数据排列:
 *            加速度计 X(2B) Y(2B) Z(2B)  ← 原始 int16
 *            陀螺仪   X(2B) Y(2B) Z(2B)  ← 原始 int16
 *          转换公式:
 *            陀螺仪 = 原始值 / 32768 * 量程(dps)     [°/s]
 *            加速度计 = 原始值 / 32768 * 量程(G)     [G]
 */
uint8_t atk_ms901m_get_gyro_accelerometer(atk_ms901m_gyro_data_t *gyro_dat,
                                           atk_ms901m_accelerometer_data_t *accel_dat,
                                           uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    float fsr;

    if ((gyro_dat == NULL) && (accel_dat == NULL)) return ATK_MS901M_ERROR;

    ret = atk_ms901m_get_frame_by_id(&frame, ATK_MS901M_FRAME_ID_GYRO_ACCE,
                                     ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) return ATK_MS901M_ERROR;

    if (accel_dat != NULL) {
        accel_dat->raw.x = (int16_t)(frame.dat[1] << 8) | frame.dat[0];
        accel_dat->raw.y = (int16_t)(frame.dat[3] << 8) | frame.dat[2];
        accel_dat->raw.z = (int16_t)(frame.dat[5] << 8) | frame.dat[4];

        fsr = (float)g_accel_fsr_table[g_accel_fsr_idx & 0x03];
        accel_dat->x = (float)accel_dat->raw.x / 32768.0f * fsr;
        accel_dat->y = (float)accel_dat->raw.y / 32768.0f * fsr;
        accel_dat->z = (float)accel_dat->raw.z / 32768.0f * fsr;
    }

    if (gyro_dat != NULL) {
        gyro_dat->raw.x = (int16_t)(frame.dat[7]  << 8) | frame.dat[6];
        gyro_dat->raw.y = (int16_t)(frame.dat[9]  << 8) | frame.dat[8];
        gyro_dat->raw.z = (int16_t)(frame.dat[11] << 8) | frame.dat[10];

        fsr = (float)g_gyro_fsr_table[g_gyro_fsr_idx & 0x03];
        gyro_dat->x = (float)gyro_dat->raw.x / 32768.0f * fsr;
        gyro_dat->y = (float)gyro_dat->raw.y / 32768.0f * fsr;
        gyro_dat->z = (float)gyro_dat->raw.z / 32768.0f * fsr;
    }

    return ATK_MS901M_EOK;
}

/*********************************************************************
 * @fn      atk_ms901m_get_magnetometer
 * @brief   等待并解析磁力计上传帧 (ID 0x04)
 *          数据: X(2B) + Y(2B) + Z(2B) + 温度(2B)
 */
uint8_t atk_ms901m_get_magnetometer(atk_ms901m_magnetometer_data_t *dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (dat == NULL) return ATK_MS901M_ERROR;

    ret = atk_ms901m_get_frame_by_id(&frame, ATK_MS901M_FRAME_ID_MAG,
                                     ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) return ATK_MS901M_ERROR;

    dat->x = (int16_t)(frame.dat[1] << 8) | frame.dat[0];
    dat->y = (int16_t)(frame.dat[3] << 8) | frame.dat[2];
    dat->z = (int16_t)(frame.dat[5] << 8) | frame.dat[4];
    dat->temperature = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 100.0f;

    return ATK_MS901M_EOK;
}

/*********************************************************************
 * @fn      atk_ms901m_get_barometer
 * @brief   等待并解析气压计上传帧 (ID 0x05)
 *          数据: 气压(4B) + 海拔(4B) + 温度(2B)
 */
uint8_t atk_ms901m_get_barometer(atk_ms901m_barometer_data_t *dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (dat == NULL) return ATK_MS901M_ERROR;

    ret = atk_ms901m_get_frame_by_id(&frame, ATK_MS901M_FRAME_ID_BARO,
                                     ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) return ATK_MS901M_ERROR;

    dat->pressure = (int32_t)(frame.dat[3] << 24) | (frame.dat[2] << 16) |
                    (frame.dat[1] << 8) | frame.dat[0];
    dat->altitude = (int32_t)(frame.dat[7] << 24) | (frame.dat[6] << 16) |
                    (frame.dat[5] << 8) | frame.dat[4];
    dat->temperature = (float)((int16_t)(frame.dat[9] << 8) | frame.dat[8]) / 100.0f;

    return ATK_MS901M_EOK;
}

/*********************************************************************
 * @fn      atk_ms901m_get_port
 * @brief   等待并解析端口数据上传帧 (ID 0x06)
 *          数据: D0(2B) + D1(2B) + D2(2B) + D3(2B)
 */
uint8_t atk_ms901m_get_port(atk_ms901m_port_data_t *dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};

    if (dat == NULL) return ATK_MS901M_ERROR;

    ret = atk_ms901m_get_frame_by_id(&frame, ATK_MS901M_FRAME_ID_PORT,
                                     ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK) return ATK_MS901M_ERROR;

    dat->d0 = (uint16_t)(frame.dat[1] << 8) | frame.dat[0];
    dat->d1 = (uint16_t)(frame.dat[3] << 8) | frame.dat[2];
    dat->d2 = (uint16_t)(frame.dat[5] << 8) | frame.dat[4];
    dat->d3 = (uint16_t)(frame.dat[7] << 8) | frame.dat[6];

    return ATK_MS901M_EOK;
}
