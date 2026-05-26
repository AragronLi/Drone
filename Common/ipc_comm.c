/********************************** (C) COPYRIGHT  *******************************
 * 文件名              : ipc_comm.c
 * 描述                : 核间通信 (IPC + HSEM) 实现
 *
 * V3F (裸机): 轮询方式, 每轮发送一帧传感器数据, MSG0-3 直传 float
 * V5F (RTOS): 中断方式收传感器, 或轮询方式收控制回传
 *******************************************************************************/
#include "ipc_comm.h"
#include "debug.h"

/*********************************************************************
 * @fn      ipc_comm_init
 * @brief   初始化 IPC 通道 + HSEM, 双核各自调用
 *          V3F: CH0=发送(传感器), CH1=接收(控制)
 *          V5F: CH0=接收(传感器), CH1=发送(控制)
 */
void ipc_comm_init(void)
{
    IPC_InitTypeDef IPC_InitStruct;

    /* 释放本核持有的全部 HSEM (复位后状态不确定) */
    HSEM_ReleaseAllSem();

#if Func_Run_V3F
    /* ==================== V3F 侧 ==================== */

    /* CH0: V3F→V5F 发送传感器数据, 裸机轮询不用中断 */
    IPC_StructInit(&IPC_InitStruct);
    IPC_InitStruct.IPC_CH = IPC_CH0;
    IPC_InitStruct.TxCID  = IPC_TxCID0;    /* V3F=0 */
    IPC_InitStruct.RxCID  = IPC_RxCID1;    /* V5F=1 */
    IPC_InitStruct.TxIER  = DISABLE;
    IPC_InitStruct.RxIER  = DISABLE;
    IPC_InitStruct.AutoEN = DISABLE;
    IPC_Init(&IPC_InitStruct);
    IPC_CH0_Lock();

    /* CH1: V5F→V3F 接收控制输出, TIM1 中断中轮询 */
    IPC_StructInit(&IPC_InitStruct);
    IPC_InitStruct.IPC_CH = IPC_CH1;
    IPC_InitStruct.TxCID  = IPC_TxCID1;
    IPC_InitStruct.RxCID  = IPC_RxCID0;
    IPC_InitStruct.TxIER  = DISABLE;
    IPC_InitStruct.RxIER  = DISABLE;
    IPC_InitStruct.AutoEN = DISABLE;
    IPC_Init(&IPC_InitStruct);
    IPC_CH1_Lock();

#else
    /* ==================== V5F 侧 ==================== */

    /* CH1: V5F→V3F 发送控制输出 */
    IPC_StructInit(&IPC_InitStruct);
    IPC_InitStruct.IPC_CH = IPC_CH1;
    IPC_InitStruct.TxCID  = IPC_TxCID1;
    IPC_InitStruct.RxCID  = IPC_RxCID0;
    IPC_InitStruct.TxIER  = DISABLE;
    IPC_InitStruct.RxIER  = DISABLE;
    IPC_InitStruct.AutoEN = DISABLE;
    IPC_Init(&IPC_InitStruct);
    IPC_CH1_Lock();

    /* CH0: V3F→V5F 接收传感器数据 (当前轮询, RTOS 就绪后改中断) */
    IPC_StructInit(&IPC_InitStruct);
    IPC_InitStruct.IPC_CH = IPC_CH0;
    IPC_InitStruct.TxCID  = IPC_TxCID0;
    IPC_InitStruct.RxCID  = IPC_RxCID1;
    IPC_InitStruct.TxIER  = DISABLE;
    IPC_InitStruct.RxIER  = DISABLE;
    IPC_InitStruct.AutoEN = DISABLE;
    IPC_Init(&IPC_InitStruct);
    IPC_CH0_Lock();
#endif

    printf("[IPC] 初始化完成\r\n");
}

/*********************************************************************
 * @fn      ipc_v3f_send_sensor_frame
 * @brief   V3F: 发送一帧传感器数据到 V5F (IPC_CH0)
 * @param   frame_id: IPC_CH0_FRAME_ATTITUDE / GYRO / ACCEL
 *          data:     传感器数据指针
 *
 * MSG 打包:
 *   ATTITUDE 帧: MSG0=roll, MSG1=pitch, MSG2=yaw, MSG3=heartbeat
 *   GYRO 帧:     MSG0=gx,   MSG1=gy,    MSG2=gz,  MSG3=heartbeat
 *   ACCEL 帧:    MSG0=ax,   MSG1=ay,    MSG2=az,  MSG3=heartbeat
 *
 * 用 HSEM_ID0 FastTake 做非阻塞保护, V5F 正在读则跳过本帧
 */
void ipc_v3f_send_sensor_frame(uint8_t frame_id,
                               const ipc_sensor_data_t *data)
{
    if (HSEM_FastTake(HSEM_ID0) != READY)
        return;

    switch (frame_id) {
    case IPC_CH0_FRAME_ATTITUDE:
        IPC_WriteMSG(IPC_MSG0, *(uint32_t *)&data->euler_roll);
        IPC_WriteMSG(IPC_MSG1, *(uint32_t *)&data->euler_pitch);
        IPC_WriteMSG(IPC_MSG2, *(uint32_t *)&data->euler_yaw);
        IPC_WriteMSG(IPC_MSG3, (data->heartbeat_ms & 0x00FFFFFF) | ((uint32_t)frame_id << 24));
        break;
    case IPC_CH0_FRAME_GYRO:
        IPC_WriteMSG(IPC_MSG0, *(uint32_t *)&data->gyro_x);
        IPC_WriteMSG(IPC_MSG1, *(uint32_t *)&data->gyro_y);
        IPC_WriteMSG(IPC_MSG2, *(uint32_t *)&data->gyro_z);
        IPC_WriteMSG(IPC_MSG3, (data->heartbeat_ms & 0x00FFFFFF) | ((uint32_t)frame_id << 24));
        break;
    case IPC_CH0_FRAME_ACCEL:
        IPC_WriteMSG(IPC_MSG0, *(uint32_t *)&data->accel_x);
        IPC_WriteMSG(IPC_MSG1, *(uint32_t *)&data->accel_y);
        IPC_WriteMSG(IPC_MSG2, *(uint32_t *)&data->accel_z);
        IPC_WriteMSG(IPC_MSG3, (data->heartbeat_ms & 0x00FFFFFF) | ((uint32_t)frame_id << 24));
        break;
    case IPC_CH0_FRAME_TOF:
        IPC_WriteMSG(IPC_MSG0, *(uint32_t *)&data->tof_altitude_m);
        IPC_WriteMSG(IPC_MSG1, data->tof_confidence);
        IPC_WriteMSG(IPC_MSG2, 0);
        IPC_WriteMSG(IPC_MSG3, (data->heartbeat_ms & 0x00FFFFFF) | ((uint32_t)frame_id << 24));
        break;
    }

    /* 通知 V5F: 数据就绪 */
    IPC_SetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);

    HSEM_ReleaseOneSem(HSEM_ID0, 1);
}

/*********************************************************************
 * @fn      ipc_v5f_recv_sensor_frame
 * @brief   V5F: 接收传感器数据帧 (IPC_CH0)
 * @param   frame_id: 输出, 帧类型
 *          data:     输出, 填充传感器数据
 *
 * 仅在 IPC_GetFlagStatus(CH0, Bit0)==SET 时调用
 */
void ipc_v5f_recv_sensor_frame(uint8_t *frame_id,
                               ipc_sensor_data_t *data)
{
    uint32_t m0 = IPC_ReadMSG(IPC_MSG0);
    uint32_t m1 = IPC_ReadMSG(IPC_MSG1);
    uint32_t m2 = IPC_ReadMSG(IPC_MSG2);
    uint32_t m3 = IPC_ReadMSG(IPC_MSG3);

    *frame_id              = (uint8_t)(m3 >> 24);
    data->heartbeat_ms     = m3 & 0x00FFFFFF;

    switch (*frame_id) {
    case IPC_CH0_FRAME_ATTITUDE:
        data->euler_roll  = *(float *)&m0;
        data->euler_pitch = *(float *)&m1;
        data->euler_yaw   = *(float *)&m2;
        break;
    case IPC_CH0_FRAME_GYRO:
        data->gyro_x = *(float *)&m0;
        data->gyro_y = *(float *)&m1;
        data->gyro_z = *(float *)&m2;
        break;
    case IPC_CH0_FRAME_ACCEL:
        data->accel_x = *(float *)&m0;
        data->accel_y = *(float *)&m1;
        data->accel_z = *(float *)&m2;
        break;
    case IPC_CH0_FRAME_TOF:
        data->tof_altitude_m = *(float *)&m0;
        data->tof_confidence  = (uint8_t)m1;
        break;
    }

    IPC_ClearFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
}

/*********************************************************************
 * @fn      ipc_v5f_send_control
 * @brief   V5F: 发送控制输出到 V3F (IPC_CH1)
 *
 * MSG0 = motor_pwm[0] | (motor_pwm[1] << 16)
 * MSG1 = motor_pwm[2] | (motor_pwm[3] << 16)
 * MSG2 = arm_state | (heartbeat_ms << 8)
 */
void ipc_v5f_send_control(const ipc_control_data_t *ctrl)
{
    if (HSEM_FastTake(HSEM_ID1) != READY)
        return;

    IPC_WriteMSG(IPC_MSG0, ctrl->motor_pwm[0] | ((uint32_t)ctrl->motor_pwm[1] << 16));
    IPC_WriteMSG(IPC_MSG1, ctrl->motor_pwm[2] | ((uint32_t)ctrl->motor_pwm[3] << 16));
    IPC_WriteMSG(IPC_MSG2, ctrl->arm_state | (ctrl->heartbeat_ms << 8));
    IPC_WriteMSG(IPC_MSG3, 0);

    IPC_SetFlagStatus(IPC_CH1, IPC_CH_Sta_Bit0);

    HSEM_ReleaseOneSem(HSEM_ID1, 1);
}

/*********************************************************************
 * @fn      ipc_v3f_recv_control
 * @brief   V3F: 接收控制输出 (IPC_CH1), 更新 PWM
 */
void ipc_v3f_recv_control(ipc_control_data_t *ctrl)
{
    if (IPC_GetFlagStatus(IPC_CH1, IPC_CH_Sta_Bit0) != SET)
        return;

    uint32_t m0 = IPC_ReadMSG(IPC_MSG0);
    uint32_t m1 = IPC_ReadMSG(IPC_MSG1);
    uint32_t m2 = IPC_ReadMSG(IPC_MSG2);

    ctrl->motor_pwm[0] = m0 & 0xFFFF;
    ctrl->motor_pwm[1] = (m0 >> 16) & 0xFFFF;
    ctrl->motor_pwm[2] = m1 & 0xFFFF;
    ctrl->motor_pwm[3] = (m1 >> 16) & 0xFFFF;
    ctrl->arm_state     = m2 & 0xFF;
    ctrl->heartbeat_ms  = m2 >> 8;

    IPC_ClearFlagStatus(IPC_CH1, IPC_CH_Sta_Bit0);
}
