/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : hardware.c
 * 作者                : WCH
 * 版本                : V1.0.1
 * 日期                : 2025/12/05
 * 描述                : 四旋翼无人机飞控硬件初始化
 *                      V3F 核心: IMU 传感器采集 (USART3) + 蜂鸣器心跳
 *                      V5F 核心: LED 心跳
 *********************************************************************************
 * Copyright (c) 2025 南京沁恒微电子股份有限公司
 *******************************************************************************/
#include "hardware.h"
#include "ipc_comm.h"
#if Func_Run_V3F
#include "atk_ms901m.h"
#include "atk_ms901m_uart.h"
#include "bsp_esc_pwm.h"
#include "bsp_tof_tfmini.h"
#endif

#if Func_Run_V3F
/* 全局传感器数据, 便于调试器观察 */
atk_ms901m_attitude_data_t      g_att;
atk_ms901m_gyro_data_t          g_gyro;
atk_ms901m_accelerometer_data_t g_accel;
#endif

/*********************************************************************
 * @fn      Hardware
 * @brief   V3F: 初始化 ATK-MS901M IMU (USART3), 循环读取姿态 +
 *               陀螺仪/加速度计数据, PB0 心跳指示
 *          V5F: 简单 LED 闪烁心跳
 */
void Hardware(void)
{
#if Func_Run_V3F
    /* ======================== V3F 核心 ======================== */
    printf("V3F 硬件初始化\r\n");

    /* ---- PB0 蜂鸣器 / 心跳指示灯 ---- */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB, ENABLE);
    {
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }

    /* ---- 初始化 ATK-MS901M IMU (USART3: PB10=TX, PB11=RX) ---- */
    uint8_t ret = atk_ms901m_init(ATK_MS901M_UART_BAUDRATE);
    if (ret != ATK_MS901M_EOK) {
        printf("[IMU] 初始化失败! 请检查接线: PB10→IMU_RX, PB11→IMU_TX, GND, 3.3V\r\n");
        while (1) {
            GPIO_ResetBits(GPIOB, GPIO_Pin_0);
            Delay_Ms(100);
            GPIO_SetBits(GPIOB, GPIO_Pin_0);
            Delay_Ms(100);
        }
    }

    printf("[IMU] 开始读取数据...\r\n");

    /* ---- 初始化 ESC PWM 输出 (TIM1 CH1-4, PA8-11, 50Hz) ---- */
    bsp_esc_pwm_init();
    printf("[ESC] PWM 初始化完成, 四路统一输出 1060us\r\n");
    bsp_esc_pwm_set_all(ESC_PWM_IDLE_US, ESC_PWM_IDLE_US,
                        ESC_PWM_IDLE_US, ESC_PWM_IDLE_US);

    /* ---- 初始化 IPC 核间通信 ---- */
    ipc_comm_init();

    /* ---- 初始化 TFmini 下视 TOF (USART4: PC11) ---- */
    bsp_tof_tfmini_init();
    printf("[TOF] TFmini 下视初始化完成\r\n");

    ipc_sensor_data_t sensor;
    ipc_control_data_t ctrl;
    tfmini_data_t tof;
    uint8_t  frame_seq = 0;      /* 姿态/陀螺/加计/TOF 轮转 */
    uint32_t tick_ms   = 0;

    while (1) {
        /* 心跳: 每次循环翻转 PB0 */
        if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0))
            GPIO_ResetBits(GPIOB, GPIO_Pin_0);
        else
            GPIO_SetBits(GPIOB, GPIO_Pin_0);

        /* 轮询姿态帧 (ID 0x01) */
        if (atk_ms901m_get_attitude(&g_att, 20) == ATK_MS901M_EOK) {
            sensor.euler_roll  = g_att.roll;
            sensor.euler_pitch = g_att.pitch;
            sensor.euler_yaw   = g_att.yaw;
        }

        /* 轮询陀螺仪 + 加速度计帧 (ID 0x03) */
        if (atk_ms901m_get_gyro_accelerometer(&g_gyro, &g_accel, 20) == ATK_MS901M_EOK) {
            sensor.gyro_x  = g_gyro.x;
            sensor.gyro_y  = g_gyro.y;
            sensor.gyro_z  = g_gyro.z;
            sensor.accel_x = g_accel.x;
            sensor.accel_y = g_accel.y;
            sensor.accel_z = g_accel.z;
        }

        /* 轮询 TFmini 下视 TOF */
        bsp_tof_tfmini_poll(&tof);
        if (tof.updated) {
            sensor.tof_altitude_m = tof.distance_mm * 0.001f;
            sensor.tof_confidence = tof.confidence;
        }

        /* 轮流发送传感器数据到 V5F */
        tick_ms += 5;
        sensor.heartbeat_ms = tick_ms;
        ipc_v3f_send_sensor_frame(frame_seq, &sensor);
        frame_seq = (frame_seq + 1) % 4;

        /* 接收 V5F 控制指令, 有更新则写入 PWM */
        ipc_v3f_recv_control(&ctrl);
        if (ctrl.motor_pwm[0] >= ESC_PWM_MIN_US) {
            bsp_esc_pwm_set_all(ctrl.motor_pwm[0], ctrl.motor_pwm[1],
                                ctrl.motor_pwm[2], ctrl.motor_pwm[3]);
        }
    }

#else
    /* ======================== V5F 核心 ======================== */
    printf("V5F 硬件初始化\r\n");

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB, ENABLE);
    {
        GPIO_InitTypeDef GPIO_InitStructure = {0};
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }

    /* ---- 初始化 IPC 核间通信 ---- */
    ipc_comm_init();

    ipc_sensor_data_t sensor;
    uint8_t  frame_id = 0;
    uint32_t print_cnt = 0;

    while (1) {
        /* PB1 心跳 */
        if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1))
            GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        else
            GPIO_SetBits(GPIOB, GPIO_Pin_1);

        /* 轮询 IPC_CH0: V3F 传感器数据 */
        if (IPC_GetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0) == SET) {
            ipc_v5f_recv_sensor_frame(&frame_id, &sensor);

            if (print_cnt++ % 50 == 0) {
                switch (frame_id) {
                case IPC_CH0_FRAME_GYRO: {
                    int32_t gx = (int32_t)(sensor.gyro_x * 100);
                    int32_t gy = (int32_t)(sensor.gyro_y * 100);
                    int32_t gz = (int32_t)(sensor.gyro_z * 100);
                    printf("[V5F] 陀螺: %ld.%02ld %ld.%02ld %ld.%02ld dps hb=%lu\r\n",
                           gx/100, (gx<0?-gx:gx)%100,
                           gy/100, (gy<0?-gy:gy)%100,
                           gz/100, (gz<0?-gz:gz)%100,
                           sensor.heartbeat_ms);
                    break;
                }
                case IPC_CH0_FRAME_ATTITUDE: {
                    int32_t r = (int32_t)(sensor.euler_roll * 100);
                    int32_t p = (int32_t)(sensor.euler_pitch * 100);
                    int32_t y = (int32_t)(sensor.euler_yaw * 100);
                    printf("[V5F] 姿态: R=%ld.%02ld P=%ld.%02ld Y=%ld.%02ld hb=%lu\r\n",
                           r/100, (r<0?-r:r)%100,
                           p/100, (p<0?-p:p)%100,
                           y/100, (y<0?-y:y)%100,
                           sensor.heartbeat_ms);
                    break;
                }
                case IPC_CH0_FRAME_ACCEL: {
                    int32_t ax = (int32_t)(sensor.accel_x * 100);
                    int32_t ay = (int32_t)(sensor.accel_y * 100);
                    int32_t az = (int32_t)(sensor.accel_z * 100);
                    printf("[V5F] 加计: %ld.%02ld %ld.%02ld %ld.%02ld G hb=%lu\r\n",
                           ax/100, (ax<0?-ax:ax)%100,
                           ay/100, (ay<0?-ay:ay)%100,
                           az/100, (az<0?-az:az)%100,
                           sensor.heartbeat_ms);
                    break;
                }
                case IPC_CH0_FRAME_TOF: {
                    int32_t mm = (int32_t)(sensor.tof_altitude_m * 1000);
                    printf("[V5F] TOF: %ld.%03ld m conf=%d hb=%lu\r\n",
                           mm/1000, mm%1000,
                           sensor.tof_confidence,
                           sensor.heartbeat_ms);
                    break;
                }
                }
            }
        }

        Delay_Ms(1);
    }
#endif
}
