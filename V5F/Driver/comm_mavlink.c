/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : comm_mavlink.c
 * 描述                : MAVLink v2 通信层实现
 *                      飞控遥测发送 + 地面站命令接收
 *******************************************************************************/

#include "comm_mavlink.h"
#include "comm_lora.h"

/* MAVLink 头文件 */
#include "common/mavlink.h"
#include "minimal/mavlink_msg_heartbeat.h"

#include <limits.h>
#include <string.h>

/* ---- MAVLink 通道状态 (channel 0) ---- */
static mavlink_status_t g_mavlink_status;
static mavlink_message_t g_mavlink_msg;

/* ---- GCS 命令 (供主循环消费) ---- */
gcs_cmd_t g_gcs_cmd;

/* ---- 系统时钟基准 (ms) ---- */
static uint32_t g_boot_ms = 0;

/* ---- 电池电压缓冲区 (10 节电芯) ---- */
static uint16_t g_bat_voltages[10];

/*********************************************************************
 * @fn      comm_mavlink_init
 * @brief   初始化 MAVLink 通信通道
 */
void comm_mavlink_init(void)
{
    memset(&g_mavlink_status, 0, sizeof(g_mavlink_status));
    memset(&g_mavlink_msg,    0, sizeof(g_mavlink_msg));
    memset(&g_gcs_cmd,        0, sizeof(g_gcs_cmd));
    g_boot_ms = 0;
}

/*********************************************************************
 * @fn      comm_mavlink_send_telemetry
 * @brief   按轮转 slot 发送 MAVLink 遥测消息
 * @param   slot        : 轮转编号 (0~7)
 * @param   sensor      : V3F 传感器数据
 * @param   arm_state   : 0=锁定 1=解锁
 * @param   flight_mode : 0=STABILIZE 1=ALTHOLD 2=AUTO
 *
 * 调度 (每 ~10ms 调用一次):
 *   slot 0: HEARTBEAT      (约 1.25Hz for all slots, 实际 ~12.5Hz per slot)
 *   slot 1: ATTITUDE
 *   slot 2: RAW_IMU
 *   slot 3: ALTITUDE
 *   slot 4: ATTITUDE (第二轮)
 *   slot 5: RAW_IMU (第二轮)
 *   slot 6: SYS_STATUS
 *   slot 7: BATTERY_STATUS
 */
void comm_mavlink_send_telemetry(uint8_t slot, const ipc_sensor_data_t *sensor,
                                  uint8_t arm_state, uint8_t flight_mode)
{
    uint8_t  buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = 0;

    g_boot_ms += 10;  /* 假定每 ~10ms 调用一次 */

    switch (slot) {

    case TELEM_SLOT_HEARTBEAT: {
        /* base_mode: bitfield */
        uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if (arm_state) {
            base_mode |= MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        }
        uint8_t system_status = arm_state ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

        mavlink_msg_heartbeat_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &g_mavlink_msg,
            MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
            base_mode, flight_mode, system_status);
        len = mavlink_msg_to_send_buffer(buf, &g_mavlink_msg);
        break;
    }

    case TELEM_SLOT_ATTITUDE:
        mavlink_msg_attitude_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &g_mavlink_msg,
            g_boot_ms,
            sensor->euler_roll,
            sensor->euler_pitch,
            sensor->euler_yaw,
            sensor->gyro_x,
            sensor->gyro_y,
            sensor->gyro_z);
        len = mavlink_msg_to_send_buffer(buf, &g_mavlink_msg);
        break;

    case TELEM_SLOT_IMU:
        /* RAW_IMU: accel in mG (milli-G), gyro in mrad/s */
        mavlink_msg_raw_imu_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &g_mavlink_msg,
            (uint64_t)g_boot_ms * 1000,  /* time_usec */
            (int16_t)(sensor->accel_x * 1000.0f),  /* xacc mG */
            (int16_t)(sensor->accel_y * 1000.0f),
            (int16_t)(sensor->accel_z * 1000.0f),
            (int16_t)(sensor->gyro_x * 1000.0f),   /* xgyro mrad/s */
            (int16_t)(sensor->gyro_y * 1000.0f),
            (int16_t)(sensor->gyro_z * 1000.0f),
            0, 0, 0,  /* xmag, ymag, zmag */
            0,         /* id */
            0);        /* temperature */
        len = mavlink_msg_to_send_buffer(buf, &g_mavlink_msg);
        break;

    case TELEM_SLOT_ALTITUDE:
        /* ALTITUDE: TOF 高度作为相对高度, 气压高度暂填 0 */
        mavlink_msg_altitude_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &g_mavlink_msg,
            g_boot_ms * 1000,                    /* time_usec */
            sensor->tof_altitude_m,              /* altitude_monotonic */
            sensor->tof_altitude_m,              /* altitude_amsl */
            sensor->tof_altitude_m,              /* altitude_local (相对起飞点) */
            sensor->tof_altitude_m,              /* altitude_relative (相对地面) */
            0.0f,                                /* altitude_terrain */
            0.0f);                               /* bottom_clearance */
        len = mavlink_msg_to_send_buffer(buf, &g_mavlink_msg);
        break;

    case TELEM_SLOT_GPS_RAW:
        mavlink_msg_gps_raw_int_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &g_mavlink_msg,
            (uint64_t)g_boot_ms * 1000,
            sensor->gps_fix_valid ? GPS_FIX_TYPE_3D_FIX : GPS_FIX_TYPE_NO_GPS,
            sensor->gps_latitude_deg_e7,
            sensor->gps_longitude_deg_e7,
            sensor->gps_altitude_mm,
            sensor->gps_hdop_x100,
            UINT16_MAX,
            (uint16_t)sensor->gps_speed_cms,
            sensor->gps_course_deg_x100,
            sensor->gps_satellites_visible,
            0,
            0,
            0,
            0,
            0,
            0);
        len = mavlink_msg_to_send_buffer(buf, &g_mavlink_msg);
        break;

    case TELEM_SLOT_GLOBAL_POS:
        mavlink_msg_global_position_int_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &g_mavlink_msg,
            g_boot_ms,
            sensor->gps_latitude_deg_e7,
            sensor->gps_longitude_deg_e7,
            sensor->gps_altitude_mm,
            sensor->gps_altitude_mm,
            0,
            0,
            0,
            sensor->compass_heading_deg_x100);
        len = mavlink_msg_to_send_buffer(buf, &g_mavlink_msg);
        break;

    case TELEM_SLOT_SYS_STATUS:
        /* SYS_STATUS: 传感器状态 + 电池 (预留, 电池参数填 0) */
        mavlink_msg_sys_status_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &g_mavlink_msg,
            MAV_SYS_STATUS_SENSOR_3D_GYRO |
            MAV_SYS_STATUS_SENSOR_3D_ACCEL |
            MAV_SYS_STATUS_SENSOR_3D_MAG |
            MAV_SYS_STATUS_SENSOR_GPS |
            MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,  /* onboard_control_sensors_present */
            MAV_SYS_STATUS_SENSOR_3D_GYRO |
            MAV_SYS_STATUS_SENSOR_3D_ACCEL |
            MAV_SYS_STATUS_SENSOR_3D_MAG |
            MAV_SYS_STATUS_SENSOR_GPS,                /* onboard_control_sensors_enabled */
            MAV_SYS_STATUS_SENSOR_3D_GYRO |
            MAV_SYS_STATUS_SENSOR_3D_ACCEL |
            ((sensor->compass_status & 0x01) ? MAV_SYS_STATUS_SENSOR_3D_MAG : 0) |
            (sensor->gps_fix_valid ? MAV_SYS_STATUS_SENSOR_GPS : 0), /* onboard_control_sensors_health */
            0,       /* load (0.1%) */
            0,       /* voltage_battery (mV) — 待 ADC 驱动实现后填充 */
            -1,      /* current_battery (10mA), -1=unknown */
            -1,      /* battery_remaining (%) */
            0, 0, 0, 0, 0, 0);
        len = mavlink_msg_to_send_buffer(buf, &g_mavlink_msg);
        break;

    case TELEM_SLOT_BATTERY:
        /* BATTERY_STATUS: 预留, 待 ADC 电池检测驱动实现后填充 */
        mavlink_msg_battery_status_pack(MAVLINK_SYS_ID, MAVLINK_COMP_ID, &g_mavlink_msg,
            0,        /* id */
            MAV_BATTERY_FUNCTION_UNKNOWN,
            MAV_BATTERY_TYPE_UNKNOWN,
            INT16_MAX, /* temperature, INT16_MAX=unknown */
            g_bat_voltages,  /* voltages[10] */
            -1,        /* current_battery (10mA) */
            -1,        /* current_consumed (mAh) */
            -1,        /* energy_consumed (hJ) */
            -1,        /* battery_remaining (%) */
            0,         /* time_remaining (s) */
            MAV_BATTERY_CHARGE_STATE_UNDEFINED,
            g_bat_voltages,  /* voltages_ext[4] */
            0,         /* mode */
            0);        /* fault_bitmask */
        len = mavlink_msg_to_send_buffer(buf, &g_mavlink_msg);
        break;

    default:
        break;
    }

    if (len > 0 && len <= MAVLINK_MAX_PACKET_LEN) {
        comm_lora_send_bytes(buf, len);
    }
}

/*********************************************************************
 * @fn      comm_mavlink_receive
 * @brief   从 LoRa 接收 FIFO 读取字节, MAVLink 状态机解帧,
 *          处理地面站命令
 */
void comm_mavlink_receive(void)
{
    while (comm_lora_available() > 0) {
        uint8_t c = comm_lora_read_byte();

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &g_mavlink_msg, &g_mavlink_status)) {
            /* 收到完整 MAVLink 消息 */
            switch (g_mavlink_msg.msgid) {

            case MAVLINK_MSG_ID_COMMAND_LONG: {
                mavlink_command_long_t cmd_long;
                mavlink_msg_command_long_decode(&g_mavlink_msg, &cmd_long);
                if (cmd_long.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                    g_gcs_cmd.arm_request = (cmd_long.param1 > 0.5f) ? 1 : 2;
                    g_gcs_cmd.last_cmd_ms = g_boot_ms;
                }
                break;
            }

            case MAVLINK_MSG_ID_SET_MODE: {
                mavlink_set_mode_t set_mode;
                mavlink_msg_set_mode_decode(&g_mavlink_msg, &set_mode);
                if (set_mode.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
                    g_gcs_cmd.mode_request = set_mode.custom_mode + 1;
                    g_gcs_cmd.last_cmd_ms = g_boot_ms;
                }
                break;
            }

            default:
                break;
            }
        }
    }
}

