# 地面站 MAVLink 通信 — 实施方案文档

> **串口分配最终确认（2026-05-31）**
> 本项目硬件分配最终以此为准：USART1=Debug(PB6/PB7)、USART2=接收机(PA3)、USART3=陀螺仪(PB10/PB11)、USART4=GPS(PC6/PC7)、USART5=PF5/PE0空闲、USART6=下视激光(PC10/PC11)、USART7=前视激光(PC12/PD2)、USART8=数传(PE8/PE7)、I2C3=罗盘(PA14=SCL/PA13=SDA)。


## 一、概述

基于 CH32H417 双核飞控，通过 数传模块（USART7, PE7/PE8, 115200 8N1）实现飞控与 PC 地面站之间的 **MAVLink v2** 双向通信。

使用标准 MAVLink 协议，地面站可直接兼容 **QGroundControl**、**Mission Planner** 等现有地面站软件。

```
PC 地面站 (Python + tkinter + pymavlink)
    │ USB-TTL (COM 口)
    ▼
数传 (地面端, USB-TTL)
    ║ 433MHz 无线
    ▼
数传 (机载端, J7: PE7/PE8)
    │ USART7 (115200 8N1)
    ▼
CH32H417 V5F 核心 (MAVLink v2 C 库)
    │ IPC (CH0: 传感器 / CH1: 控制指令)
    ▼
V3F 核心 → IMU/TOF/光流/ESC PWM
```

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

## 二、新增文件清单

### 2.1 MAVLink 协议库

| 路径 | 说明 |
|------|------|
| `Middleware/MAVLink/` | MAVLink v2 C 头文件库 (header-only) |
| ├─ `protocol.h` | 核心协议定义 (帧格式、版本号) |
| ├─ `mavlink_types.h` | 基础类型定义 |
| ├─ `mavlink_helpers.h` | 核心 API (`mavlink_parse_char`, `mavlink_msg_to_send_buffer` 等) |
| ├─ `mavlink_conversions.h` | 字节序转换 |
| ├─ `checksum.h` | CRC16 校验 |
| ├─ `common/` | 通用消息集 (205 个消息类型) |
| ├─ `standard/` | 标准消息集 |
| └─ `minimal/` | 最小消息集 (HEARTBEAT 等) |

> 通过 pymavlink 从 `common.xml` / `standard.xml` / `minimal.xml` 自动生成。

### 2.2 飞控端驱动

| 文件 | 说明 |
|------|------|
| `V5F/Driver/comm_lora.h` | USART8 数传驱动头文件 |
| `V5F/Driver/comm_lora.c` | USART8 驱动实现 (FIFO + RXNE 中断) |
| `V5F/Driver/comm_mavlink.h` | MAVLink 通信层头文件 |
| `V5F/Driver/comm_mavlink.c` | MAVLink 遥测发送 + 命令接收实现 |

### 2.3 地面站应用

| 文件 | 说明 |
|------|------|
| `GroundStation/requirements.txt` | Python 依赖 (pyserial, pymavlink, matplotlib) |
| `GroundStation/gcs_main.py` | 主入口, tkinter 三栏布局 GUI |
| `GroundStation/gcs_mavlink.py` | MAVLink 串口通信管理 (pymavlink 封装) |
| `GroundStation/gcs_dashboard.py` | 仪表盘 (人工地平仪 + 姿态/高度/电池/状态显示) |
| `GroundStation/gcs_plot.py` | 6 通道实时波形 (matplotlib) |
| `GroundStation/gcs_control.py` | 控制面板 (ARM/DISARM + 飞行模式切换) |

---

## 三、修改文件清单

### 3.1 Common/hardware.c

**V3F 分支改动：**

| 位置 | 改动 | 说明 |
|------|------|------|
| L20-21 | `+ #include "sensor_flow.h"` | 引入光流传感器驱动 |
| L30 | `+ opFlow_t g_flow;` | 全局光流数据结构 |
| L82-83 | `+ sensor_flow_init()` | 初始化 ATK-PMW3901 光流 (SPI3) |
| L122-132 | `+ 光流轮询代码块` | 读取光流 Δx/Δy 并写入 sensor 结构体 |
| L135 | `frame_seq % 4 → % 5` | IPC 帧轮转从 4 帧扩展为 5 帧 (增加光流帧) |

**V5F 分支改动：**

| 位置 | 改动 | 说明 |
|------|------|------|
| L19-21 | `+ #include "comm_lora.h"` `"comm_mavlink.h"` | 引入数传和 MAVLink 驱动 |
| L151-152 | `+ #define ESC_PWM_MIN_US / IDLE_US` | 本地 PWM 常量 (V5F 无法访问 V3F 驱动头文件) |
| L167-173 | `+ comm_lora_init / comm_mavlink_init` | 初始化 数传 USART8 和 MAVLink 通信层 |
| L179-180 | `+ arm_state / flight_mode 变量` | 飞控状态机变量 |
| L194 | `+ 精简调试打印` | 只保留 Attitude 和 TOF 帧打印 |
| L199-203 | `+ 遥测轮转发送` | 每 10ms 发送一帧 MAVLink 遥测 |
| L205-207 | `+ comm_mavlink_receive()` | 解析地面站命令 |
| L209-226 | `+ GCS 命令消费` | 处理 ARM/DISARM 和模式切换 |
| L228-249 | `+ IPC_CH1 控制输出` | 向 V3F 发送电机 PWM 控制指令 |

### 3.2 V5F/User/ch32h417_it.c

| 位置 | 改动 | 说明 |
|------|------|------|
| L13 | `+ #include "comm_lora.h"` | 引入 数传 驱动中断处理函数声明 |
| L18 | `+ USART7_IRQHandler 声明` | WCH 快速中断属性声明 |
| L61-68 | `+ USART7_IRQHandler 实现` | 中断入口, 转发到 `COMM_LORA_UART_IRQHandler()` |

---

## 四、通信协议详解

### 4.1 MAVLink v2 帧结构

```
STX  LEN  INCOMPAT_FLAGS  COMPAT_FLAGS  SEQ  SYSID  COMPID  MSGID(3B)  PAYLOAD  CHECKSUM(2B)  SIGNATURE(13B, 可选)
1B   1B   1B              1B            1B   1B     1B      3B         N bytes  2B            13B
```

- **STX**: `0xFD` (MAVLink v2 帧头)
- **总开销**: 12 字节 (不含 SIGNATURE)
- **最大载荷**: 255 字节

### 4.2 下行遥测 (飞控 → 地面站)

| 轮转槽位 | MAVLink 消息 | MSG_ID | 数据内容 | 实际频率 |
|---------|-------------|--------|---------|---------|
| 0 | HEARTBEAT | #0 | type=QUADROTOR, autopilot=GENERIC, base_mode, system_status | ~1.25 Hz |
| 1 | ATTITUDE | #30 | roll, pitch, yaw (rad), rollspeed, pitchspeed, yawspeed (rad/s) | ~1.25 Hz |
| 2 | RAW_IMU | #27 | xacc/yacc/zacc (mG), xgyro/ygyro/zgyro (mrad/s) | ~1.25 Hz |
| 3 | ALTITUDE | #141 | altitude_monotonic/amsl/local/relative/terrain (m) | ~1.25 Hz |
| 4 | ATTITUDE | #30 | (第二轮, 提高姿态刷新率) | ~1.25 Hz |
| 5 | RAW_IMU | #27 | (第二轮) | ~1.25 Hz |
| 6 | SYS_STATUS | #1 | sensors_present/enabled/health, voltage, current | ~1.25 Hz |
| 7 | BATTERY_STATUS | #147 | battery_function/type/temperature/voltages/remaining | ~1.25 Hz |

> 8 帧 × 10ms = 80ms 周期, 单消息等效约 12.5Hz (实际 HEARTBEAT 约 1.25Hz)

### 4.3 上行命令 (地面站 → 飞控)

| MAVLink 消息 | 用途 | 参数 |
|-------------|------|------|
| COMMAND_LONG (#76) | ARM/DISARM | cmd=400 (MAV_CMD_COMPONENT_ARM_DISARM), param1: 1=ARM, 0=DISARM |
| SET_MODE (#11) | 切换飞行模式 | base_mode=1 (CUSTOM_MODE_ENABLED), custom_mode: 0=STABILIZE, 1=ALTHOLD, 2=AUTO |

### 4.4 系统 ID 分配

| 设备 | System ID | Component ID |
|------|-----------|-------------|
| 飞控 (V5F) | 1 | 1 (MAV_COMP_ID_AUTOPILOT1) |
| 地面站 (GCS) | 255 | 190 (MAV_COMP_ID_MISSIONPLANNER) |

---

## 五、USART8 驱动说明

### 5.1 引脚映射

| 信号 | 引脚 | GPIO AF | 总线 |
|------|------|---------|------|
| USART7 TX | PE8 | AF7 | HB1 |
| USART7 RX | PE7 | AF7 | HB1 |
| GPIO 时钟 | PE7/PE8 | — | HB2 |

### 5.2 配置参数

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 流控 | 无 |
| RX FIFO 大小 | 512 字节 (环形缓冲区) |
| 中断 | RXNE (接收缓冲区非空) |

### 5.3 API

```c
void     comm_lora_init(uint32_t baudrate);         // 初始化 USART7
void     comm_lora_send_bytes(const uint8_t *dat, uint16_t len);  // 阻塞发送
uint16_t comm_lora_available(void);                  // FIFO 可读字节数
uint8_t  comm_lora_read_byte(void);                  // 从 FIFO 读一个字节
void     comm_lora_flush(void);                      // 清空 FIFO
void     COMM_LORA_UART_IRQHandler(void);            // 中断处理 (由 ch32h417_it.c 调用)
```

---

## 六、MAVLink 通信层说明

### 6.1 遥测发送

```c
void comm_mavlink_send_telemetry(uint8_t slot,
                                  const ipc_sensor_data_t *sensor,
                                  uint8_t arm_state,
                                  uint8_t flight_mode);
```

- `slot`: 轮转编号 (0~7)
- `sensor`: 来自 V3F IPC_CH0 的最新传感器数据
- `arm_state`: 0=锁定, 1=解锁
- `flight_mode`: 0=STABILIZE, 1=ALTHOLD, 2=AUTO

内部调用 `mavlink_msg_*_pack()` 组帧 → `mavlink_msg_to_send_buffer()` 序列化 → `comm_lora_send_bytes()` 发送。

### 6.2 命令接收

```c
void comm_mavlink_receive(void);  // 在主循环中调用, 非阻塞
```

内部使用 MAVLink 状态机 `mavlink_parse_char()` 逐字节解析:

1. 从 `comm_lora_read_byte()` 取字节
2. `mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)` 组帧
3. 收到完整帧后根据 `msg.msgid` 分发:
   - `MAVLINK_MSG_ID_COMMAND_LONG (#76)` → 解析 ARM/DISARM
   - `MAVLINK_MSG_ID_SET_MODE (#11)` → 解析飞行模式切换
4. 设置 `g_gcs_cmd` 标志位供主循环消费

### 6.3 命令标志结构

```c
typedef struct {
    uint8_t  arm_request;    // 0=无请求, 1=ARM, 2=DISARM
    uint8_t  mode_request;   // 0=无请求, 1=STABILIZE, 2=ALTHOLD, 3=AUTO
    uint32_t last_cmd_ms;    // 最后收到命令的时间戳
} gcs_cmd_t;
```

---

## 七、地面站使用说明

### 7.1 环境要求

- Python 3.8+
- Windows / Linux / macOS

### 7.2 安装

```bash
cd GroundStation
pip install -r requirements.txt
```

### 7.3 启动

```bash
python gcs_main.py
```

### 7.4 界面布局

```
┌─────────────────────────────────────────────────────────┐
│ [串口 ▼] [刷新] [波特率 ▼] [连接/断开]  未连接  RX: 0  │
├──────────┬────────────────────────┬─────────────────────┤
│ 姿态地平仪│                        │   飞行控制           │
│ (Canvas) │   实时波形              │   [ARM 解锁]        │
│          │   Roll    Pitch         │   [DISARM 锁定]     │
│ Roll:0°  │   Yaw     Altitude      │                     │
│ Pitch:0° │   Battery GyroX         │   飞行模式:         │
│ Yaw:0°   │                        │   ○ STABILIZE      │
│ Alt:0m   │   (matplotlib)          │   ○ ALTHOLD        │
│ Bat:--   │                        │   ○ AUTO            │
│ Status   │                        │                     │
│ Mode     │                        │   命令日志           │
│ HBeat    │                        │   ┌───────────┐     │
│ ● 连接灯  │                        │   │           │     │
└──────────┴────────────────────────┴─────────────────────┘
```

### 7.5 操作流程

1. USB-TTL 模块插入 PC, 另一端接 数传 模块 (地面端)
2. 飞控上电, 机载 数传 模块自动连接
3. 点击 **刷新** 扫描串口, 选择对应 COM 口
4. 波特率选择 **115200**, 点击 **连接**
5. 连接成功: 状态灯变绿, 心跳计数开始, 姿态/波形实时更新
6. 点击 **ARM 解锁** → 飞控输出怠速 PWM (1060us)
7. 选择飞行模式 → 飞控切换控制模式
8. 点击 **DISARM 锁定** → 飞控输出停转 PWM (1000us)

---

## 八、数据流详解

### 8.1 传感器数据路径 (上行遥测)

```
V3F 主循环 (~200Hz)
  ├── atk_ms901m_get_attitude()     → euler_roll/pitch/yaw
  ├── atk_ms901m_get_gyro_accel()   → gyro_x/y/z, accel_x/y/z
  ├── bsp_tof_tfmini_poll()         → tof_altitude_m, confidence
  └── sensor_flow_poll()            → flow_dx, flow_dy, quality
       │
       ▼ ipc_v3f_send_sensor_frame() 分 5 帧轮转
       │ (IPC_CH0, V3F → V5F)
       ▼
V5F 主循环 (~1kHz)
  ├── ipc_v5f_recv_sensor_frame()   ← 拼合到 g_sensor
  │
  ├── comm_mavlink_send_telemetry() 每 ~10ms
  │     ├── mavlink_msg_*_pack()    组帧
  │     ├── mavlink_msg_to_send_buffer()  序列化
  │     └── comm_lora_send_bytes()  通过 USART7 发出
  │
  └── ipc_v5f_send_control()        ← 控制指令到 V3F
       (IPC_CH1, V5F → V3F)
```

### 8.2 控制指令路径 (下行命令)

```
地面站 GUI
  ├── ARM 解锁 → conn.mav.command_long_send(MAV_CMD_COMPONENT_ARM_DISARM, param1=1)
  └── 模式切换 → conn.mav.set_mode_send(base_mode, custom_mode)
       │
       ▼ pymavlink → 串口 → USB-TTL → 数传 无线
       │
V5F USART7 RX 中断
  ├── COMM_LORA_UART_IRQHandler()   → FIFO 写入
  │
V5F 主循环
  ├── comm_mavlink_receive()        → mavlink_parse_char()
  ├── g_gcs_cmd.arm_request = 1/2   → 主循环消费
  ├── g_gcs_cmd.mode_request = 1/2/3
  │
  └── ipc_v5f_send_control()        → IPC_CH1
       │
V3F 主循环
  └── ipc_v3f_recv_control()        → bsp_esc_pwm_set_all()
```

---

## 九、配置参数汇总

### 9.1 飞控端

```c
// comm_mavlink.h
#define MAVLINK_SYS_ID      1       // 飞控系统 ID
#define MAVLINK_COMP_ID     1       // 飞控组件 ID
#define TELEM_NUM_SLOTS     8       // 遥测轮转槽位数

// comm_lora.h
#define COMM_LORA_UART_BAUDRATE  115200  // USART7 波特率
#define COMM_LORA_UART_RX_FIFO_SIZE  512  // 接收 FIFO 大小
```

### 9.2 地面站端

```python
# gcs_mavlink.py
MAVLINK_SYS_ID = 255    # 地面站系统 ID
MAVLINK_COMP_ID = 190   # 地面站组件 ID

# gcs_main.py
UPDATE_INTERVAL_MS = 50  # GUI 更新周期
```

### 9.3 波形窗口

```python
# gcs_plot.py
WINDOW_SEC = 30   # 滑动窗口 30 秒
MAX_POINTS = 600  # 最大数据点数
```

---

## 十、编译指南 (飞控端)

### 10.1 添加 Include Path

在 V5F 工程中添加以下路径:

```
../Middleware/MAVLink
../V5F/Driver
```

### 10.2 添加源文件

将以下 `.c` 文件加入 V5F 工程编译列表:

```
../V5F/Driver/comm_lora.c
../V5F/Driver/comm_mavlink.c
```

### 10.3 中断向量表

确保启动文件中包含 `USART7_IRQHandler` 向量 (CH32H417 官方启动文件默认已包含)。

---

## 十一、调试建议

### 11.1 飞控端调试

- 用 USB-TTL 连接 PC 和 USART7 (PE7/PE8)，串口助手 115200 查看 MAVLink 原始帧
- 帧头 `0xFD` 为 MAVLink v2 帧，`HEARTBEAT` 消息 MSG_ID=0 可用于验证发送
- 若 USART7 无输出，检查 `RCC_HB1Periph_USART7` 时钟使能和 GPIO AF 配置

### 11.2 地面站调试

- 先用 USB-TTL 直连飞控 USART8 (不经 数传) 验证有线通信
- 确认串口助手中能看到 `0xFD` 开头的 MAVLink 帧后再启动地面站
- 地面站控制台 (stdout) 会打印连接/接收错误信息

### 11.3 常见问题

| 问题 | 可能原因 | 排查方向 |
|------|---------|---------|
| USART7 无输出 | 时钟未使能 | 检查 RCC_HB1Periph_USART7 / RCC_HB2Periph_GPIOE |
| 地面站连不上 | COM 口选错 | 设备管理器确认 USB-TTL 端口号 |
| 收到帧但无数据显示 | MAVLink SYS_ID 不匹配 | 飞控 SYS_ID=1, 地面站过滤目标 sysid=1 |
| 数传 通信断断续续 | 天线未接/波特率不匹配 | 确认两端波特率均为 115200 |

---

## 十二、后续扩展方向

1. **GPS 位置遥测**: 添加 `GLOBAL_POSITION_INT (#33)` 消息, 显示经纬度/高度/速度
2. **航点上传**: 实现 `MISSION_ITEM (#39)` / `MISSION_COUNT (#44)` 协议, 支持航点规划
3. **PID 参数调参**: 实现 `PARAM_REQUEST_LIST (#21)` / `PARAM_SET (#23)`, 实时调整 PID 参数
4. **SD 卡日志回传**: 通过 `FILE_TRANSFER_PROTOCOL (#110)` 回传飞行日志
5. **电池电压 ADC 驱动**: 完善 `SYS_STATUS` 和 `BATTERY_STATUS` 中的电压/电流真实数据
6. **空闲/预留 链路**: 将 空闲/预留 (USART8) 同样接入 MAVLink, 实现双链路冗余
