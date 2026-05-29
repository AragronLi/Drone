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
#include "sensor_flow.h"
#else
#include "comm_lora.h"
#include "comm_mavlink.h"
#endif

#if Func_Run_V3F
/* 全局传感器数据, 便于调试器观察 */
atk_ms901m_attitude_data_t      g_att;
atk_ms901m_gyro_data_t          g_gyro;
atk_ms901m_accelerometer_data_t g_accel;
opFlow_t                        g_flow;
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

    /* ---- 初始化 ATK-PMW3901 光流 (SPI3: PB3/4/5 + PC5/CS) ---- */
    sensor_flow_init();

    ipc_sensor_data_t sensor = {0};
    ipc_control_data_t ctrl;
    tfmini_data_t tof;
    uint8_t  frame_seq = 0;      /* 姿态/陀螺/加计/TOF/光流 轮转 */
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

        /* 轮询 ATK-PMW3901 光流 (~100Hz, dt≈0.01s) */
        {
            float height     = sensor.tof_altitude_m;
            float roll_rad   = g_att.roll  * 0.0174533f;  /* DEG2RAD */
            float pitch_rad  = g_att.pitch * 0.0174533f;
            sensor_flow_poll(&g_flow, height, roll_rad, pitch_rad, 0.01f);
            sensor.flow_dx      = g_flow.deltaPos[0];
            sensor.flow_dy      = g_flow.deltaPos[1];
            sensor.flow_quality = 0;  /* squal 由 sensor_flow_poll 内部使用, 此处暂不暴露 */
            sensor.flow_updated = g_flow.isDataValid ? 1 : 0;
        }

        /* 轮流发送传感器数据到 V5F */
        tick_ms += 5;
        sensor.heartbeat_ms = tick_ms;
        ipc_v3f_send_sensor_frame(frame_seq, &sensor);
        frame_seq = (frame_seq + 1) % 5;

        /* 接收 V5F 控制指令, 有更新则写入 PWM */
        ipc_v3f_recv_control(&ctrl);
        if (ctrl.motor_pwm[0] >= ESC_PWM_MIN_US) {
            bsp_esc_pwm_set_all(ctrl.motor_pwm[0], ctrl.motor_pwm[1],
                                ctrl.motor_pwm[2], ctrl.motor_pwm[3]);
        }
    }

#else
    /* ======================== V5F 核心 ======================== */
    #define ESC_PWM_MIN_US   1000   /* 停转脉宽 us */
    #define ESC_PWM_IDLE_US  1060   /* 怠速脉宽 us */

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

    /* ---- 初始化 LoRa 数传 (USART7) ---- */
    comm_lora_init(COMM_LORA_UART_BAUDRATE);
    printf("[LoRa] USART7 初始化完成, 波特率 %d\r\n", COMM_LORA_UART_BAUDRATE);

    /* ---- 初始化 MAVLink 通信层 ---- */
    comm_mavlink_init();
    printf("[MAVLink] 通信层初始化完成, SYS_ID=%d\r\n", MAVLINK_SYS_ID);

    ipc_sensor_data_t sensor;   /* 最新完整传感器数据 (各帧拼合) */
    ipc_control_data_t ctrl;    /* 发往 V3F 的控制指令 */
    uint8_t  frame_id = 0;
    uint32_t loop_cnt  = 0;
    uint8_t  telem_slot = 0;
    uint8_t  arm_state   = 0;   /* 0=锁定 1=解锁 */
    uint8_t  flight_mode = 0;   /* 0=STABILIZE 1=ALTHOLD 2=AUTO */

    while (1) {
        /* PB1 心跳 */
        if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_1))
            GPIO_ResetBits(GPIOB, GPIO_Pin_1);
        else
            GPIO_SetBits(GPIOB, GPIO_Pin_1);

        /* 轮询 IPC_CH0: V3F 传感器数据, 拼合到 sensor 结构体 */
        if (IPC_GetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0) == SET) {
            ipc_v5f_recv_sensor_frame(&frame_id, &sensor);

            /* 每 100 帧打印一次调试信息 */
            if (loop_cnt % 100 == 0) {
                switch (frame_id) {
                case IPC_CH0_FRAME_ATTITUDE:
                    printf("[V5F] Att: R=%.2f P=%.2f Y=%.2f\r\n",
                           sensor.euler_roll, sensor.euler_pitch, sensor.euler_yaw);
                    break;
                case IPC_CH0_FRAME_TOF:
                    printf("[V5F] TOF: %.3fm conf=%d\r\n",
                           sensor.tof_altitude_m, sensor.tof_confidence);
                    break;
                }
            }
        }

        /* ---- 每 ~10ms 发送一帧 MAVLink 遥测 (8 帧轮转, 80ms 周期) ---- */
        if (loop_cnt % 10 == 0) {
            comm_mavlink_send_telemetry(telem_slot, &sensor, arm_state, flight_mode);
            telem_slot = (telem_slot + 1) % TELEM_NUM_SLOTS;
        }

        /* ---- 处理地面站命令 ---- */
        comm_mavlink_receive();

        /* 消费 GCS 命令 */
        if (g_gcs_cmd.arm_request) {
            if (g_gcs_cmd.arm_request == 1) {
                arm_state = 1;
                printf("[GCS] ARM 解锁\r\n");
            } else {
                arm_state = 0;
                printf("[GCS] DISARM 锁定\r\n");
            }
            g_gcs_cmd.arm_request = 0;
        }
        if (g_gcs_cmd.mode_request) {
            flight_mode = g_gcs_cmd.mode_request - 1;
            if (flight_mode > 2) flight_mode = 0;
            printf("[GCS] 模式切换: %d\r\n", flight_mode);
            g_gcs_cmd.mode_request = 0;
        }

        /* ---- 发送控制指令到 V3F (通过 IPC_CH1) ---- */
        ctrl.arm_state    = arm_state;
        ctrl.heartbeat_ms = loop_cnt;
        if (arm_state) {
            /* 解锁时输出怠速 PWM (1060us), 后续由 PID 混控器接管 */
            ctrl.motor_pwm[0] = ESC_PWM_IDLE_US;
            ctrl.motor_pwm[1] = ESC_PWM_IDLE_US;
            ctrl.motor_pwm[2] = ESC_PWM_IDLE_US;
            ctrl.motor_pwm[3] = ESC_PWM_IDLE_US;
        } else {
            /* 锁定时输出停转 PWM (1000us) */
            ctrl.motor_pwm[0] = ESC_PWM_MIN_US;
            ctrl.motor_pwm[1] = ESC_PWM_MIN_US;
            ctrl.motor_pwm[2] = ESC_PWM_MIN_US;
            ctrl.motor_pwm[3] = ESC_PWM_MIN_US;
        }
        ipc_v5f_send_control(&ctrl);

        loop_cnt++;
        Delay_Ms(1);
    }
#endif
}
