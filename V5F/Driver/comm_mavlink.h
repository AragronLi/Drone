/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : comm_mavlink.h
 * 描述                : MAVLink v2 通信层头文件
 *                      飞控 MAVLink 遥测发送 + 地面站命令接收
 *******************************************************************************/

#ifndef __COMM_MAVLINK_H
#define __COMM_MAVLINK_H

#include "ch32h417.h"
#include "ipc_comm.h"

/* ---- MAVLink 系统参数 ---- */
#define MAVLINK_SYS_ID      1       /* 飞控系统 ID */
#define MAVLINK_COMP_ID     1       /* 飞控组件 ID (MAV_COMP_ID_AUTOPILOT1) */

/* ---- 遥测轮转调度 (每 10ms 发一帧, 8 帧一轮 = 80ms 周期) ---- */
#define TELEM_SLOT_HEARTBEAT    0   /* 1Hz */
#define TELEM_SLOT_ATTITUDE     1   /* ~12.5Hz */
#define TELEM_SLOT_IMU          2   /* ~12.5Hz */
#define TELEM_SLOT_ALTITUDE     3   /* ~12.5Hz */
#define TELEM_SLOT_GPS_RAW      4   /* GPS_RAW_INT */
#define TELEM_SLOT_GLOBAL_POS   5   /* GLOBAL_POSITION_INT */
#define TELEM_SLOT_SYS_STATUS   6   /* ~12.5Hz */
#define TELEM_SLOT_BATTERY      7   /* ~12.5Hz */
#define TELEM_NUM_SLOTS         8

/* ---- GCS 命令回调标志 (主循环消费) ---- */
typedef struct {
    uint8_t  arm_request;       /* 0=无请求, 1=ARM, 2=DISARM */
    uint8_t  mode_request;      /* 0=无请求, 1=STABILIZE, 2=ALTHOLD, 3=AUTO */
    uint32_t last_cmd_ms;       /* 最后收到命令的时间戳 */
} gcs_cmd_t;

extern gcs_cmd_t g_gcs_cmd;

/* ---- 公开接口 ---- */
void comm_mavlink_init(void);
void comm_mavlink_send_telemetry(uint8_t slot, const ipc_sensor_data_t *sensor,
                                  uint8_t arm_state, uint8_t flight_mode);
void comm_mavlink_receive(void);

#endif /* __COMM_MAVLINK_H */
