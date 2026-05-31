/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : ipc_comm.h
 * 描述                : 核间通信 (IPC + HSEM) 共享数据结构
 *
 * 通道分配:
 *   IPC_CH0  V3F → V5F  传感器数据 (MSG0-3)
 *   IPC_CH1  V5F → V3F  控制输出   (MSG0-3)
 *
 * HSEM 分配:
 *   HSEM_ID0  V3F 写保护
 *   HSEM_ID1  V5F 写保护
 *******************************************************************************/
#ifndef __IPC_COMM_H
#define __IPC_COMM_H

#include "ch32h417.h"

/* ============================================================
 *  V3F → V5F: 传感器数据 (通过 IPC_CH0 的 4 个 MSG)
 * ============================================================ */
typedef struct {
    float    euler_roll;
    float    euler_pitch;
    float    euler_yaw;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    tof_altitude_m;
    uint8_t  tof_confidence;
    float    flow_dx;          /* 光流 X 位移 (cm) */
    float    flow_dy;          /* 光流 Y 位移 (cm) */
    uint8_t  flow_quality;     /* squal 表面质量 0~255 */
    uint8_t  flow_updated;     /* 光流数据是否更新 */

    uint8_t  gps_fix_valid;
    uint8_t  gps_fix_quality;
    uint8_t  gps_fix_type;
    uint8_t  gps_satellites_used;
    uint8_t  gps_satellites_visible;
    int32_t  gps_latitude_deg_e7;
    int32_t  gps_longitude_deg_e7;
    int32_t  gps_altitude_mm;
    uint16_t gps_hdop_x100;
    uint32_t gps_speed_cms;
    uint16_t gps_course_deg_x100;

    int16_t  compass_mag_x;
    int16_t  compass_mag_y;
    int16_t  compass_mag_z;
    uint16_t compass_heading_deg_x100;
    uint8_t  compass_status;

    uint32_t heartbeat_ms;
} ipc_sensor_data_t;

/* ============================================================
 *  V5F → V3F: 控制输出 (通过 IPC_CH1 的 4 个 MSG)
 * ============================================================ */
typedef struct {
    uint16_t motor_pwm[4];   /* 1000~2000 us */
    uint8_t  arm_state;      /* 0=锁定 1=解锁 */
    uint32_t heartbeat_ms;
} ipc_control_data_t;

/* ---- IPC MSG 打包格式 ---- */

/* CH0 MSG (V3F→V5F): 分 5 帧轮流发送传感器子集
 * MSG3 = heartbeat_ms[23:0] | (frame_id << 24) */
#define IPC_CH0_FRAME_ATTITUDE   0   /* roll/pitch/yaw + heartbeat */
#define IPC_CH0_FRAME_GYRO       1   /* gyro_x/y/z + heartbeat */
#define IPC_CH0_FRAME_ACCEL      2   /* accel_x/y/z + heartbeat */
#define IPC_CH0_FRAME_TOF        3   /* tof_altitude_m + confidence + heartbeat */
#define IPC_CH0_FRAME_FLOW       4   /* flow_dx + flow_dy + quality + updated + heartbeat */
#define IPC_CH0_FRAME_GPS_POS    5   /* lat/lon/alt + heartbeat */
#define IPC_CH0_FRAME_GPS_STAT   6   /* fix/sats/hdop + heartbeat */
#define IPC_CH0_FRAME_GPS_VEL    7   /* speed/course + heartbeat */
#define IPC_CH0_FRAME_COMPASS    8   /* mag/heading + heartbeat */

/* ---- API ---- */
void ipc_comm_init(void);
void ipc_v3f_send_sensor_frame(uint8_t frame_id,
                               const ipc_sensor_data_t *data);
void ipc_v5f_send_control(const ipc_control_data_t *ctrl);
void ipc_v3f_recv_control(ipc_control_data_t *ctrl);
void ipc_v5f_recv_sensor_frame(uint8_t *frame_id,
                               ipc_sensor_data_t *data);

#endif /* __IPC_COMM_H */
