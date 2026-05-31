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
#include "sensor_gps.h"
#include "sensor_compass.h"
#include "oled_ssd1306.h"
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
gps_data_t                      g_gps;
compass_data_t                  g_compass;

static int32_t abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static void format_deg_e7(char *buf, uint8_t len, int32_t deg_e7)
{
    int32_t abs_val = abs_i32(deg_e7);
    snprintf(buf, len, "%c%ld.%07ld",
             (deg_e7 < 0) ? '-' : '+',
             (long)(abs_val / 10000000),
             (long)(abs_val % 10000000));
}

static const char *gps_fix_type_text(uint8_t fix_type)
{
    if (fix_type == 3) return "3D";
    if (fix_type == 2) return "2D";
    return "NO";
}

static void oled_show_gps_page(const gps_data_t *gps)
{
    char line[24];
    char coord[16];
    uint32_t utc = gps->utc_time_ms / 1000;

    oled_clear();

    snprintf(line, sizeof(line), "GPS %c %s Q%u S%02u/%02u",
             gps->fix_valid ? 'A' : 'V',
             gps_fix_type_text(gps->fix_type),
             gps->fix_quality,
             gps->satellites_used,
             gps->satellites_visible);
    oled_show_string(0, 0, line, FONT_6X8_WIDTH);

    format_deg_e7(coord, sizeof(coord), gps->latitude_deg_e7);
    snprintf(line, sizeof(line), "LAT %s", coord);
    oled_show_string(0, 8, line, FONT_6X8_WIDTH);

    format_deg_e7(coord, sizeof(coord), gps->longitude_deg_e7);
    snprintf(line, sizeof(line), "LON %s", coord);
    oled_show_string(0, 16, line, FONT_6X8_WIDTH);

    snprintf(line, sizeof(line), "ALT %ld.%01ldm",
             (long)(gps->altitude_mm / 1000),
             (long)(abs_i32(gps->altitude_mm % 1000) / 100));
    oled_show_string(0, 24, line, FONT_6X8_WIDTH);

    snprintf(line, sizeof(line), "SPD %lu.%02lum/s",
             (unsigned long)(gps->speed_cms / 100),
             (unsigned long)(gps->speed_cms % 100));
    oled_show_string(0, 32, line, FONT_6X8_WIDTH);

    snprintf(line, sizeof(line), "CRS %u.%02u HD %u.%02u",
             gps->course_deg_x100 / 100,
             gps->course_deg_x100 % 100,
             gps->hdop_x100 / 100,
             gps->hdop_x100 % 100);
    oled_show_string(0, 40, line, FONT_6X8_WIDTH);

    snprintf(line, sizeof(line), "UTC %02lu:%02lu:%02lu",
             (unsigned long)((utc / 3600) % 24),
             (unsigned long)((utc / 60) % 60),
             (unsigned long)(utc % 60));
    oled_show_string(0, 48, line, FONT_6X8_WIDTH);

    snprintf(line, sizeof(line), "NMEA %lu OK %lu",
             (unsigned long)gps->sentence_count,
             (unsigned long)gps->checksum_ok_count);
    oled_show_string(0, 56, line, FONT_6X8_WIDTH);

    oled_refresh();
}
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

    /* ---- 初始化 TFmini 下视 TOF (USART6: PC11) ---- */
    bsp_tof_tfmini_init();
    printf("[TOF] TFmini 下视初始化完成\r\n");

    /* ---- 初始化 ATK-PMW3901 光流 (SPI3: PB3/4/5 + PC5/CS) ---- */
    sensor_flow_init();

    /* ---- 初始化 SR2525M10D GPS (USART4: PC6/PC7) ---- */
    sensor_gps_init(SENSOR_GPS_UART_BAUDRATE);
    printf("[GPS] SR2525M10D USART4 初始化完成, 波特率 %d\r\n", SENSOR_GPS_UART_BAUDRATE);

    /* ---- 初始化 0.96寸 OLED (SSD1306, I2C3: PA14=SCL, PA13=SDA) ---- */
    oled_init();
    oled_show_string(0, 0, "GPS OLED READY", FONT_6X8_WIDTH);
    oled_show_string(0, 8, "I2C3 PA14/PA13", FONT_6X8_WIDTH);
    oled_refresh();
    printf("[OLED] SSD1306 初始化完成, 与罗盘共享 I2C3 PA14/PA13\r\n");

    /* ---- 初始化 SR2525M10D 罗盘 QMC5883L (I2C3: PA14=SCL, PA13=SDA) ---- */
    // compass_init();
    // printf("[COMPASS] QMC5883L 初始化完成\r\n");

    ipc_sensor_data_t sensor = {0};
    ipc_control_data_t ctrl;
    tfmini_data_t tof;
    uint8_t  frame_seq = 0;      /* 高速传感器轮转 */
    uint8_t  slow_frame_seq = 0; /* GPS/罗盘低频轮转 */
    uint32_t tick_ms   = 0;
    uint32_t oled_tick  = 0;
    uint32_t tof_print_tick = 0;
    uint32_t tof_irq_print_tick = 0;
    uint32_t tof_irq_count_last = 0;
    uint32_t tof_poll_count_last = 0;
    uint32_t pc10_edge_count = 0;
    uint32_t pc11_edge_count = 0;
    uint8_t pc10_last = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);
    uint8_t pc11_last = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11);
    uint32_t imu_print_tick = 0;
    uint32_t gps_print_tick = 0;
    uint32_t compass_print_tick = 0;

    while (1) {
        /* 心跳: 每次循环翻转 PB0 */
        if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0))
            GPIO_ResetBits(GPIOB, GPIO_Pin_0);
        else
            GPIO_SetBits(GPIOB, GPIO_Pin_0);

        // /* 轮询姿态帧 (ID 0x01) */
        // if (atk_ms901m_get_attitude(&g_att, 20) == ATK_MS901M_EOK) {
        //     sensor.euler_roll  = g_att.roll;
        //     sensor.euler_pitch = g_att.pitch;
        //     sensor.euler_yaw   = g_att.yaw;
        // }

        // /* 轮询陀螺仪 + 加速度计帧 (ID 0x03) */
        // if (atk_ms901m_get_gyro_accelerometer(&g_gyro, &g_accel, 20) == ATK_MS901M_EOK) {
        //     sensor.gyro_x  = g_gyro.x;
        //     sensor.gyro_y  = g_gyro.y;
        //     sensor.gyro_z  = g_gyro.z;
        //     sensor.accel_x = g_accel.x;
        //     sensor.accel_y = g_accel.y;
        //     sensor.accel_z = g_accel.z;

        //     /* 限频打印: 每约 200ms 输出一次陀螺仪/加速度计数据 */
        //     if (++imu_print_tick >= 40) {
        //         imu_print_tick = 0;
        //         /* newlib-nano 默认不支持 printf 浮点, 这里转成 x1000 整数打印 */
        //         //printf("[IMU-GYRO] gx=%ld gy=%ld gz=%ld, ax=%ld ay=%ld az=%ld (x1000)\r\n",
        //                (long)(sensor.gyro_x * 1000.0f),
        //                (long)(sensor.gyro_y * 1000.0f),
        //                (long)(sensor.gyro_z * 1000.0f),
        //                (long)(sensor.accel_x * 1000.0f),
        //                (long)(sensor.accel_y * 1000.0f),
        //                (long)(sensor.accel_z * 1000.0f)/*)*/;
        //     }
        // }

        // /* 轮询 TFmini 下视 TOF */
        // {
        //     uint8_t pc10_now = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10);
        //     uint8_t pc11_now = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11);
        //     if (pc10_now != pc10_last) {
        //         pc10_last = pc10_now;
        //         pc10_edge_count++;
        //     }
        //     if (pc11_now != pc11_last) {
        //         pc11_last = pc11_now;
        //         pc11_edge_count++;
        //     }
        // }

        // if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == SET) {
        //     g_tof_poll_count++;
        //     g_tof_poll_last_byte = (uint8_t)USART_ReceiveData(USART6);
        // }

        // if (++tof_irq_print_tick >= 200) {
        //     tof_irq_print_tick = 0;
        //     printf("[TOF-IRQ] irq=%lu, irq_delta=%lu, poll=%lu, poll_delta=%lu, irq_last=0x%02X, poll_last=0x%02X, PC10=%d edge=%lu, PC11=%d edge=%lu\r\n",
        //            g_tof_irq_count,
        //            g_tof_irq_count - tof_irq_count_last,
        //            g_tof_poll_count,
        //            g_tof_poll_count - tof_poll_count_last,
        //            g_tof_last_rx_byte,
        //            g_tof_poll_last_byte,
        //            GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10),
        //            pc10_edge_count,
        //            GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11),
        //            pc11_edge_count);
        //     tof_irq_count_last = g_tof_irq_count;
        //     tof_poll_count_last = g_tof_poll_count;
        // }

        // bsp_tof_tfmini_poll(&tof);
        // if (tof.updated) {
        //     sensor.tof_altitude_m = tof.distance_mm * 0.001f;
        //     sensor.tof_confidence = tof.confidence;

        //     /* 限频打印: 每约 200ms 输出一次下视激光测距数据 */
        //     if (++tof_print_tick >= 40) {
        //         tof_print_tick = 0;
        //         printf("[TOF-DOWN] distance=%d mm, altitude=%ld mm, confidence=%d\r\n",
        //                tof.distance_mm,
        //                (long)(sensor.tof_altitude_m * 1000.0f),
        //                tof.confidence);
        //     }
        // }

        // /* 轮询 ATK-PMW3901 光流 (~100Hz, dt≈0.01s) */
        // {
        //     float height     = sensor.tof_altitude_m;
        //     float roll_rad   = g_att.roll  * 0.0174533f;  /* DEG2RAD */
        //     float pitch_rad  = g_att.pitch * 0.0174533f;
        //     sensor_flow_poll(&g_flow, height, roll_rad, pitch_rad, 0.01f);
        //     sensor.flow_dx      = g_flow.deltaPos[0];
        //     sensor.flow_dy      = g_flow.deltaPos[1];
        //     sensor.flow_quality = 0;  /* squal 由 sensor_flow_poll 内部使用, 此处暂不暴露 */
        //     sensor.flow_updated = g_flow.isDataValid ? 1 : 0;
        // }

        /* 轮询 SR2525M10D GPS NMEA0183 */
        sensor_gps_poll();
        if (sensor_gps_get_data(&g_gps)) {
            sensor.gps_fix_valid          = g_gps.fix_valid;
            sensor.gps_fix_quality        = g_gps.fix_quality;
            sensor.gps_fix_type           = g_gps.fix_type;
            sensor.gps_satellites_used    = g_gps.satellites_used;
            sensor.gps_satellites_visible = g_gps.satellites_visible;
            sensor.gps_latitude_deg_e7    = g_gps.latitude_deg_e7;
            sensor.gps_longitude_deg_e7   = g_gps.longitude_deg_e7;
            sensor.gps_altitude_mm        = g_gps.altitude_mm;
            sensor.gps_hdop_x100          = g_gps.hdop_x100;
            sensor.gps_speed_cms          = g_gps.speed_cms;
            sensor.gps_course_deg_x100    = g_gps.course_deg_x100;

            if (++gps_print_tick >= 20) {
                gps_print_tick = 0;
                printf("[GPS] fix=%d qual=%d sats=%d lat=%ld lon=%ld alt=%ldmm spd=%lu course=%u.%02u\r\n",
                       sensor.gps_fix_valid,
                       sensor.gps_fix_quality,
                       sensor.gps_satellites_used,
                       sensor.gps_latitude_deg_e7,
                       sensor.gps_longitude_deg_e7,
                       sensor.gps_altitude_mm,
                       sensor.gps_speed_cms,
                       sensor.gps_course_deg_x100 / 100,
                       sensor.gps_course_deg_x100 % 100);
            }
        }

        // /* 轮询 SR2525M10D QMC5883L 罗盘 (与 OLED 共享 I2C3, 限频到约 25Hz) */
        // if ((tick_ms % 40) == 0 && compass_read_data(&g_compass)) {
        //     g_compass.heading = compass_get_heading(g_compass.mag_x, g_compass.mag_y, 0.0f);
        //     sensor.compass_mag_x = g_compass.mag_x;
        //     sensor.compass_mag_y = g_compass.mag_y;
        //     sensor.compass_mag_z = g_compass.mag_z;
        //     sensor.compass_heading_deg_x100 = (uint16_t)(g_compass.heading * 100.0f);
        //     sensor.compass_status = g_compass.status;

        //     if (++compass_print_tick >= 25) {
        //         compass_print_tick = 0;
        //         printf("[COMPASS] x=%d y=%d z=%d heading=%u.%02u\r\n",
        //                sensor.compass_mag_x,
        //                sensor.compass_mag_y,
        //                sensor.compass_mag_z,
        //                sensor.compass_heading_deg_x100 / 100,
        //                sensor.compass_heading_deg_x100 % 100);
        //     }
        // }

        /* 轮流发送传感器数据到 V5F: 高频帧保持 5 帧轮转, GPS/罗盘低频插入 */
        tick_ms += 5;
        sensor.heartbeat_ms = tick_ms;
        ipc_v3f_send_sensor_frame(frame_seq, &sensor);
        frame_seq = (frame_seq + 1) % 5;

        if ((tick_ms % 50) == 0) {
            ipc_v3f_send_sensor_frame(IPC_CH0_FRAME_GPS_POS + slow_frame_seq, &sensor);
            slow_frame_seq = (slow_frame_seq + 1) % 4;
        }

         /* OLED 全屏刷新占用 I2C 总线, 限频到 1Hz 避免影响 GPS 解析和罗盘读取 */
         if ((tick_ms - oled_tick) >= 1000) {
             oled_tick = tick_ms;
            oled_show_gps_page(&g_gps);
        }

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

    /* ---- 初始化 LoRa 数传 (USART8: PE8/PE7) ---- */
    comm_lora_init(COMM_LORA_UART_BAUDRATE);
    printf("[LoRa] USART8 初始化完成, 波特率 %d\r\n", COMM_LORA_UART_BAUDRATE);

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
