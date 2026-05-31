# CH32H417 大小核 IPC 信息共享实现

> **串口分配最终确认（2026-05-31）**
> 本项目硬件分配最终以此为准：USART1=Debug(PB6/PB7)、USART2=接收机(PA3)、USART3=陀螺仪(PB10/PB11)、USART4=GPS(PC6/PC7)、USART5=PF5/PE0空闲、USART6=下视激光(PC10/PC11)、USART7=前视激光(PC12/PD2)、USART8=数传(PE8/PE7)、I2C3=罗盘(PA14=SCL/PA13=SDA)。


## 一、概述

CH32H417 双核（V3F 120MHz 小核 + V5F 480MHz 大核）通过**两套硬件外设**配合实现核间信息共享：

| 外设 | 基地址 | 功能 | 特点 |
|------|--------|------|------|
| **IPC** | `0xE000D000` | 数据传输通道 | 4 通道 × 4 个 32 位 MSG，带中断，双向标志位通知 |
| **HSEM** | `0xE000C000` | 硬件互斥锁 | 32 个信号量，支持两步/快速获取，记录占用核心 ID |

---

### 串口分配最终表

| 串口 | 功能 | 引脚分配 | 说明 |
|------|------|----------|------|
| USART1 | Debug 调试串口 | PB6/PB7 | 115200，调试输出 |
| USART2 | 接收机 | PA3 | 115200，单线 RX |
| USART3 | 陀螺仪/IMU | PB10/PB11 | 460800，姿态/角速度/加速度 |
| USART4 | GPS | PC6/PC7 | 38400（默认），NMEA |
| USART5 | 空闲/预留 | PF5/PE0 | 当前不接外设 |
| USART6 | 下视激光 | PC10/PC11 | 115200，定高测距 |
| USART7 | 前视激光 | PC12/PD2 | 115200，前向避障 |
| USART8 | 数传 | PE8/PE7 | 115200，地面站/MAVLink |

---

## 二、IPC 外设 — 数据传输通道

### 2.1 寄存器布局

定义于 [core_riscv.h](SRC/Core/core_riscv.h#L99-L110)：

```c
typedef struct {
    __IO uint32_t CTLR;      // 通道控制寄存器 (4ch × 8bit)
    __I  uint32_t ISR;       // 中断状态寄存器
    __I  uint32_t ISM;       // 中断屏蔽状态 (只读)
    uint32_t RESERVED0;
    __IO uint32_t ENA;       // 中断使能
    __IO uint32_t STS;       // 通道状态标志 (双向置位/查询)
    __O  uint32_t SET;       // 硬件置位 STS (写1有效)
    __O  uint32_t CLR;       // 硬件清零 STS (写1有效)
    __IO uint32_t MSG[4];    // 4 个 32 位消息寄存器 (双核均可读写)
} IPC_Type;

#define IPC ((IPC_Type *) 0xE000D000)
```

### 2.2 通道控制寄存器 (CTLR)

4 个通道，每个通道占 8 位：

| 通道 | CTLR 位段 | 说明 |
|------|----------|------|
| CH0 | CTLR[7:0] | 通道 0 配置 |
| CH1 | CTLR[15:8] | 通道 1 配置 |
| CH2 | CTLR[23:16] | 通道 2 配置 |
| CH3 | CTLR[31:24] | 通道 3 配置 |

**单通道 8 位字段：**

| Bit | 名称 | 功能 |
|-----|------|------|
| 1:0 | TxCID | 发送方核心 ID（0=V3F, 1=V5F） |
| 3:2 | RxCID | 接收方核心 ID |
| 4 | TxIER | 发送中断使能 |
| 5 | RxIER | 接收中断使能 |
| 6 | AutoEN | 自动使能模式 |
| 7 | LOCK | 锁定通道配置 (写1后不可再改) |

### 2.3 状态标志位 (STS)

每个通道 8 个用户可用的标志位（STS Bit0~Bit7），双核均可通过 SET/CLR 寄存器操作：

```
发送方: IPC_SetFlagStatus(CH, BitX)   → STS 对应位置 1
接收方: IPC_GetFlagStatus(CH, BitX)   → 轮询检测
         IPC_ClearFlagStatus(CH, BitX) → 清除标志
```

### 2.4 中断

每个通道有独立的中断向量（中断号 16-19）：

| 中断 | 中断号 | 说明 |
|------|--------|------|
| `IPC_CH0_IRQn` | 16 | 通道 0 中断 |
| `IPC_CH1_IRQn` | 17 | 通道 1 中断 |
| `IPC_CH2_IRQn` | 18 | 通道 2 中断 |
| `IPC_CH3_IRQn` | 19 | 通道 3 中断 |

### 2.5 核心 API

定义于 [ch32h417_ipc.h](SRC/Peripheral/inc/ch32h417_ipc.h) / [ch32h417_ipc.c](SRC/Peripheral/src/ch32h417_ipc.c)：

```c
// 初始化
void IPC_DeInit(void);
void IPC_Init(IPC_InitTypeDef* IPC_InitStruct);
void IPC_StructInit(IPC_InitTypeDef* IPC_InitStruct);

// 锁定通道配置
void IPC_CH0_Lock(void);  // 同时也支持 CH1/CH2/CH3

// 中断管理
void IPC_ITConfig(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit, FunctionalState NewState);
ITStatus IPC_GetITStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);
ITStatus IPC_GetITMask(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);

// 标志位操作
FlagStatus IPC_GetFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);
void IPC_SetFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);
void IPC_ClearFlagStatus(IPC_Channel_TypeDef IPC_CH, IPC_ChannelStateBit_TypeDef TPC_Sta_Bit);

// 消息读写
void IPC_WriteMSG(IPC_MSG_TypeDef IPC_MSG, uint32_t Data);   // MSG0~MSG3
uint32_t IPC_ReadMSG(IPC_MSG_TypeDef IPC_MSG);
```

---

## 三、HSEM 外设 — 硬件信号量

### 3.1 寄存器布局

定义于 [core_riscv.h](SRC/Core/core_riscv.h#L78-L96)：

```c
typedef struct {
    __IO uint32_t RX[32];      // 32 个信号量寄存器 (bit31=锁位, bit8=CoreID, bit7:0=ProcessID)
    uint8_t RESERVED0[0x80];
    __I  uint32_t RLRX[32];    // 快速获取只读镜像
    uint8_t RESERVED1[0x80];
    __IO uint32_t LSE;         // 所有信号量锁定状态 (每 bit 对应一个信号量)
    uint8_t RESERVED2[4];
    __IO uint32_t CLR;         // 清除寄存器 (需配合 KEY 写入)
    __IO uint32_t KEY;         // 密钥寄存器 (默认值 0x5AA50000)
    uint8_t RESERVED3[0xF0];
    __IO uint32_t IER;         // 中断使能
    __IO uint32_t ISR;         // 中断状态
    __I  uint32_t ISM;         // 中断屏蔽状态 (只读)
    __I  uint32_t LSM;         // 当前核持有的信号量掩码
} HSEM_Type;

#define HSEM ((HSEM_Type *) 0xE000C000)
```

### 3.2 信号量寄存器位定义

| Bit | 含义 |
|-----|------|
| 31 | Lock — 1=已锁定, 0=空闲 |
| 8 | CoreID — 占用方核心 ID（0=V3F, 1=V5F） |
| 7:0 | ProcessID — 占用方进程 ID（0~255，用户自定义） |

### 3.3 核心 ID 定义

```c
#define HSEM_Core_ID_V3F   ((uint32_t)0x00000000)
#define HSEM_Core_ID_V5F   ((uint32_t)0x00000100)
```

### 3.4 核心 API

定义于 [ch32h417_hsem.h](SRC/Peripheral/inc/ch32h417_hsem.h) / [ch32h417_hsem.c](SRC/Peripheral/src/ch32h417_hsem.c)：

```c
// 获取信号量
ErrorStatus HSEM_Take(HSEM_ID_TypeDef HSEM_ID, uint32_t ProcessID);       // 两步模式 (写 + 回读验证)
ErrorStatus HSEM_FastTake(HSEM_ID_TypeDef HSEM_ID);                        // 单步快速获取

// 释放信号量
void HSEM_ReleaseOneSem(HSEM_ID_TypeDef HSEM_ID, uint32_t ProcessID);
void HSEM_ReleaseAllSem(void);                                              // 释放本核持有的全部
void HSEM_ReleaseSem_MatchCID_PID(uint32_t CoreID, uint32_t ProcessID);
void HSEM_ReleaseSem_MatchCID(uint32_t CoreID);
void HSEM_ReleaseSem_MatchPID(uint32_t ProcessID);

// 状态查询
FunctionalState HSEM_GetOneSemTakenState(HSEM_ID_TypeDef HSEM_ID);
uint32_t HSEM_GetAllSemTakenState(void);                                    // 所有核
uint32_t HSEM_OwnCoreGetAllSemTakenState(void);                             // 仅本核

// 中断管理
void HSEM_ITConfig(HSEM_ID_TypeDef HSEM_ID, FunctionalState NewState);
FlagStatus HSEM_GetFlagStatus(HSEM_ID_TypeDef HSEM_ID);
void HSEM_ClearFlag(HSEM_ID_TypeDef HSEM_ID);
ITStatus HSEM_GetITStatus(HSEM_ID_TypeDef HSEM_ID);
void HSEM_ClearITPendingBit(HSEM_ID_TypeDef HSEM_ID);

// 密钥管理
void HSEM_SetClearKey(uint32_t Key);
uint32_t HSEM_GetClearKey(void);
```

> **注意**：HSEM 只有一个总中断向量 `HSEM_IRQn = 28`，32 个信号量共享，需在 ISR 中轮询判断具体是哪个信号量触发。

---

## 四、飞控项目中的实际应用

### 4.1 IPC 通道分配

| 通道 | 方向 | 用途 | 频率 |
|------|------|------|------|
| IPC_CH0 | V3F→V5F | 传感器数据 (MSG0~3 轮流) | 200Hz |
| IPC_CH1 | V5F→V3F | 控制输出 (motor_pwm / arm_state / buzzer) | 1kHz |
| IPC_CH2 | 双向 | 心跳 / 握手 | 1kHz |

### 4.2 HSEM 信号量分配

| 信号量 | 用途 | 持有方 |
|--------|------|--------|
| HSEM_ID0 | 传感器数据共享缓冲区保护 | V3F (写) / V5F (读) |
| HSEM_ID1 | 控制输出共享缓冲区保护 | V5F (写) / V3F (读) |

### 4.3 共享数据结构

```c
// V3F → V5F: 传感器数据
typedef struct {
    // ATK IMU (USART3, 200Hz)
    float    euler_roll, euler_pitch, euler_yaw;
    float    gyro_x, gyro_y, gyro_z;       // 角速率 rad/s
    float    accel_x, accel_y, accel_z;     // 加速度 m/s^2
    float    baro_altitude;                 // 气压高度 m
    uint32_t imu_timestamp_ms;

    // 下视激光 (USART4, 100Hz)
    float    tof_altitude;
    uint8_t  tof_updated;

    // ATK 光流 (SPI3, 100Hz)
    float    flow_dx, flow_dy;              // 位移 cm
    float    flow_vel_x, flow_vel_y;        // 速度 cm/s
    uint8_t  flow_quality;
    uint8_t  flow_updated;

    // IA6B IBUS (USART2, ~140Hz)
    uint16_t rc_channels[14];              // 1000~2000
    uint8_t  rc_updated;

    // 电池 (ADC1, 10Hz)
    float    battery_voltage;
    float    battery_current;

    // 心跳
    uint32_t v3f_heartbeat_ms;
} v3f_to_v5f_t;

// V5F → V3F: 控制输出
typedef struct {
    uint16_t motor_pwm[4];                 // 1000~2000 μs
    uint8_t  motor_updated;
    uint8_t  arm_state;                    // 0=锁定 1=解锁
    uint16_t buzzer_freq;                  // 0=关闭
    uint32_t v5f_heartbeat_ms;
} v5f_to_v3f_t;
```

### 4.4 数据流拓扑

```
V3F (裸机, 120MHz)                        V5F (FreeRTOS, 480MHz)
┌──────────────────────┐                  ┌─────────────────────┐
│                      │  IPC_CH0 (MSG0-3)│                     │
│ USART2 IBUS ────┐    │ ───────────────→ │ vTaskFusion (1kHz)  │
│ USART3 IMU ─────┤    │  传感器数据       │ vTaskControl (1kHz) │
│ USART6 下视激光 ─┤    │                  │ vTaskAltitude       │
│ SPI3 光流 ──────┤    │                  │ vTaskNavigation     │
│ ADC1 电池 ──────┤    │                  │                     │
│ 罗盘旧 I2C3 方案已作废 ──────┘    │                  │                     │
│                      │  IPC_CH1 (MSG0-3)│                     │
│ TIM1 PWM ← ESC×4    │ ←─────────────── │ motor_pwm[4]        │
│ GPIO 蜂鸣器          │   控制输出        │ arm_state           │
│                      │                  │                     │
│         IPC_CH2 (MSG0) ←──心跳/握手──→ IPC_CH2 (MSG0)        │
└──────────────────────┘                  └─────────────────────┘
```

### 4.5 典型通信流程示例

**V3F 发送传感器数据到 V5F：**

```c
// ========== V3F 侧 (发送方) ==========
void v3f_send_sensor_data(void)
{
    // 使用 HSEM 保护共享内存写入
    if (HSEM_Take(HSEM_ID0, 1) == READY) {
        // 方式 A: 大数据块 — 写入共享 SRAM
        memcpy(&shared_sram->sensors, &local_sensor_data, sizeof(v3f_to_v5f_t));

        // 方式 B: 小数据 — 直接通过 MSG 传递 float 值
        IPC_WriteMSG(IPC_MSG0, *(uint32_t*)&local_sensor_data.euler_roll);
        IPC_WriteMSG(IPC_MSG1, *(uint32_t*)&local_sensor_data.euler_pitch);
        IPC_WriteMSG(IPC_MSG2, *(uint32_t*)&local_sensor_data.euler_yaw);

        // 通知 V5F 数据就绪
        IPC_SetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);

        HSEM_ReleaseOneSem(HSEM_ID0, 1);
    }
}

// ========== V5F 侧 (接收方) ==========
// 方式 1: 中断方式
void IPC_CH0_IRQHandler(void)
{
    if (IPC_GetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0) == SET) {
        // 释放信号量给 FreeRTOS 任务
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sensor_data_ready_sem, &xHigherPriorityTaskWoken);

        IPC_ClearFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// 方式 2: 轮询方式 (在 FreeRTOS 任务中)
void vTaskSensorRead(void *pvParameters)
{
    while (1) {
        if (IPC_GetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0) == SET) {
            float roll  = *(float*)&IPC_ReadMSG(IPC_MSG0);
            float pitch = *(float*)&IPC_ReadMSG(IPC_MSG1);
            float yaw   = *(float*)&IPC_ReadMSG(IPC_MSG2);

            IPC_ClearFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);

            // 送入姿态解算
            madgwick_update(roll, pitch, yaw, gyro, accel);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
```

**V5F 发送控制指令到 V3F：**

```c
// ========== V5F 侧 (发送方) ==========
void v5f_send_control_output(void)
{
    HSEM_Take(HSEM_ID1, 1);

    IPC_WriteMSG(IPC_MSG0, motor_pwm[0] | (motor_pwm[1] << 16));
    IPC_WriteMSG(IPC_MSG1, motor_pwm[2] | (motor_pwm[3] << 16));
    IPC_WriteMSG(IPC_MSG2, (arm_state << 24) | v5f_heartbeat_ms);
    IPC_SetFlagStatus(IPC_CH1, IPC_CH_Sta_Bit0);

    HSEM_ReleaseOneSem(HSEM_ID1, 1);
}

// ========== V3F 侧 (接收方, 在 TIM1 中断中更新 PWM) ==========
void TIM1_UP_IRQHandler(void)
{
    if (IPC_GetFlagStatus(IPC_CH1, IPC_CH_Sta_Bit0) == SET) {
        uint32_t m0m1 = IPC_ReadMSG(IPC_MSG0);
        uint32_t m2m3 = IPC_ReadMSG(IPC_MSG1);
        uint32_t ctrl = IPC_ReadMSG(IPC_MSG2);

        motor_pwm[0] = m0m1 & 0xFFFF;
        motor_pwm[1] = m0m1 >> 16;
        motor_pwm[2] = m2m3 & 0xFFFF;
        motor_pwm[3] = m2m3 >> 16;

        IPC_ClearFlagStatus(IPC_CH1, IPC_CH_Sta_Bit0);
    }

    // 更新 TIM1 CCR 寄存器
    TIM_SetCompare1(TIM1, motor_pwm[0]);
    TIM_SetCompare2(TIM1, motor_pwm[1]);
    TIM_SetCompare3(TIM1, motor_pwm[2]);
    TIM_SetCompare4(TIM1, motor_pwm[3]);
}
```

---

## 五、初始化示例

### 5.1 IPC 通道初始化 (V3F + V5F 双方都要配)

```c
void IPC_Config(void)
{
    IPC_InitTypeDef IPC_InitStruct;

#if Func_Run_V3F
    /* V3F → V5F: CH0 发送传感器数据 */
    IPC_StructInit(&IPC_InitStruct);
    IPC_InitStruct.IPC_CH = IPC_CH0;
    IPC_InitStruct.TxCID  = IPC_TxCID0;    // V3F=0 发送
    IPC_InitStruct.RxCID  = IPC_RxCID1;    // V5F=1 接收
    IPC_InitStruct.TxIER  = DISABLE;        // V3F 裸机轮询, 不用中断
    IPC_InitStruct.RxIER  = DISABLE;
    IPC_Init(IPC_InitStruct);

    /* V5F → V3F: CH1 接收控制输出 */
    IPC_InitStruct.IPC_CH = IPC_CH1;
    IPC_InitStruct.TxCID  = IPC_TxCID1;    // V5F=1 发送
    IPC_InitStruct.RxCID  = IPC_RxCID0;    // V3F=0 接收
    IPC_InitStruct.RxIER  = DISABLE;        // TIM1 中断中轮询
    IPC_Init(IPC_InitStruct);

#else
    /* V5F → V3F: CH1 发送控制输出 */
    IPC_InitStruct.IPC_CH = IPC_CH1;
    IPC_InitStruct.TxCID  = IPC_TxCID1;
    IPC_InitStruct.RxCID  = IPC_RxCID0;
    IPC_InitStruct.TxIER  = DISABLE;
    IPC_InitStruct.RxIER  = DISABLE;
    IPC_Init(IPC_InitStruct);

    /* V3F → V5F: CH0 接收传感器数据 (中断方式) */
    IPC_InitStruct.IPC_CH = IPC_CH0;
    IPC_InitStruct.TxCID  = IPC_TxCID0;
    IPC_InitStruct.RxCID  = IPC_RxCID1;
    IPC_InitStruct.RxIER  = ENABLE;         // V5F 用中断唤醒任务
    IPC_Init(IPC_InitStruct);

    // 使能 NVIC 中断
    NVIC_EnableIRQ(IPC_CH0_IRQn);
#endif
}
```

### 5.2 HSEM 初始化

```c
void HSEM_Config(void)
{
    // 确保本核释放所有信号量（复位后可能处于不确定状态）
    HSEM_ReleaseAllSem();
}
```

---

## 六、设计要点

1. **高频小数据优先用 MSG 直传**：4 个 32 位寄存器足够传 4 个 float 或打包多个 uint16，省去共享内存管理开销。

2. **大数据块用共享 SRAM + 指针**：对于完整的 `v3f_to_v5f_t` 结构体（~80 字节），应放在双核均可访问的 SRAM 区域，MSG 只传偏移量或指针，HSEM 保护并发。

3. **V5F 建议中断驱动**：V5F 运行 FreeRTOS，IPC 中断（`IPC_CH0_IRQn`）可配合 `xSemaphoreGiveFromISR` 高效唤醒任务。V3F 裸机侧轮询即可。

4. **心跳超时保护**：双方各维护 `heartbeat_ms` 时间戳，对方核 500ms 无更新则触发安全动作（V3F 强制停转 PWM、V5F 进入 FAILSAFE）。

5. **HSEM 避免死锁**：
   - 获取信号量后**尽快释放**，不在持锁期间做耗时操作
   - 使用 `HSEM_FastTake` 做非阻塞尝试，失败则下次重试
   - 飞控场景下推荐 **单写者模式**：每个数据块只有一个核写入，避免写冲突

6. **通道锁定**：初始化完成后调用 `IPC_CHx_Lock()` 锁定通道配置，防止误修改导致通信错乱。

---

## 七、相关源文件

| 文件 | 说明 |
|------|------|
| [SRC/Core/core_riscv.h](SRC/Core/core_riscv.h) | IPC/HSEM 寄存器结构体定义 + 基地址宏 |
| [SRC/Peripheral/inc/ch32h417_ipc.h](SRC/Peripheral/inc/ch32h417_ipc.h) | IPC 外设 API 声明 |
| [SRC/Peripheral/src/ch32h417_ipc.c](SRC/Peripheral/src/ch32h417_ipc.c) | IPC 外设 API 实现 |
| [SRC/Peripheral/inc/ch32h417_hsem.h](SRC/Peripheral/inc/ch32h417_hsem.h) | HSEM 外设 API 声明 |
| [SRC/Peripheral/src/ch32h417_hsem.c](SRC/Peripheral/src/ch32h417_hsem.c) | HSEM 外设 API 实现 |
| [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md) | 飞控系统双核分工 + IPC 数据流 |

---

## 八、参考资料

- CH32H417 参考手册（RM）- IPC / HSEM 章节
- CH32H417 数据手册 - 双核启动与核间通信
- [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md) 第五节 — 核间通信 (IPC)
