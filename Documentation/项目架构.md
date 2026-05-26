# 四旋翼无人机飞控系统架构文档

## 一、系统概述

基于青稞 RISC-V MCU 的双核无人机飞控系统。CH32H417 作为飞控核心，CH32V203 驱动 4 路无刷电机 ESC（原方案不改动）。支持定高飞行、航点规划、前向避障，可选边缘 AI 视觉。

### 1.1 选题要求满足情况

| 要求 | 状态 | 实现方式 |
|------|------|---------|
| ESC 使用 CH32V203 | ✅ 必选 | 原方案不改，六步换相，PWM 输入 |
| 飞控使用 CH32H417 | ✅ 必选 | 双核，V3F 采集 + V5F 算法 |
| 定高飞行 | ✅ 必选 | TFmini 下视 TOF + IMU 气压融合 |
| 航点规划 | ✅ 必选 | GPS + 罗盘 → L1 路径跟随 |
| 高速电机优化 | ✅ 鼓励 | 2216 1400KV + 自适应进角 + 换相加速 |
| 飞控动力协同 | ✅ 鼓励 | 电池电压前馈补偿 |
| 边缘 AI | 🔲 可选 | OpenMV H7+ 预留 USART5 |

### 1.2 整机配置

| 部件 | 型号 | 数量 |
|------|------|------|
| 飞控 MCU | CH32H417 | 1 |
| ESC | CH32V203 六步换相 | 4 |
| 电机 | 2216 1400KV | 4 |
| 桨 | 9443 两叶 | 4 |
| 电池 | 4S 5200mAh | 1 |
| 机架 | F450 (450mm) | 1 |
| 遥控器 | Flysky i6X + IA6B/IA10B (IBUS) | 1 |
| 起飞重量 | ~1360g | — |
| 推重比 | ~2.8:1 | — |

---

## 二、硬件架构

### 2.1 系统框图

```
                          ┌──────────────────────────┐
                          │    CH32H417 飞控核心板     │
                          │                          │
IA6B 接收机 ──IBUS─→ PA3  │  USART2 RX               │
ATK 十轴IMU ──USART→ PB11 │  USART3 RX               │
TFmini 下视 ──USART→ PC11 │  USART4 RX               │
TFmini 前向 ──USART→ PD2  │  USART5 RX               │
ATK 光流    ──SPI──→ PB3-5│  SPI3                    │
GPS+罗盘    ──USART→ PC6/7│  USART6 TX/RX            │
	罗盘        ──I2C──→ PB8/9│  I2C1 SCL/SDA            │
	OLED 小屏   ──I2C──→ PB8/9│  I2C1 SCL/SDA (共享)     │
                          │                          │
ATK LoRa    ──USART→ PE7/8│  USART7 TX/RX  数传      │
ESP8266     ──USART→ PE0/1│  USART8 TX/RX  WiFi      │
Debug       ──USART→ PB6  │  USART1 TX               │
                          │                          │
ESC ×4      ←─PWM── PA8-11│  TIM1 CH1-4              │
蜂鸣器      ←─GPIO─ PB0   │                          │
LED 红      ←─GPIO─ PB1   │                          │
                          │                          │
SD 卡       ←─SPI── PB12-15│ SPI2                    │
电池电压    ←─ADC── PA1    │  ADC1_IN1 (分压 6:1)     │
电池电流    ←─ADC── PA2    │  ADC1_IN2 (ACS712)       │
                          └──────────────────────────┘
```

### 2.2 引脚分配表

#### 执行器

| 引脚 | 功能 | 设备 | 插座 |
|------|------|------|------|
| PA8 | TIM1_CH1 | ESC 1 PWM | J10-2 |
| PA9 | TIM1_CH2 | ESC 2 PWM | J10-3 |
| PA10 | TIM1_CH3 | ESC 3 PWM | J10-4 |
| PA11 | TIM1_CH4 | ESC 4 PWM | J10-5 |
| PB0 | GPIO | 蜂鸣器 | J11 |
| PB1 | GPIO | LED 红 (板载) | — |

#### 传感器

| 引脚 | 外设 | 设备 | 插座 | 速率 |
|------|------|------|------|------|
| PA3 | USART2_RX | IA6B IBUS 接收机 | J1 | 115200 |
| PA1 | ADC1_IN1 | 电池电压 | — | 10Hz |
| PA2 | ADC1_IN2 | 电池电流 | — | 10Hz |
| PB11 | USART3_RX | ATK 十轴 IMU | J2 | 460800 |
| PC11 | USART4_RX | TFmini 下视 TOF | J3 | 115200 |
| PD2 | USART5_RX | TFmini 前向 TOF | J4 | 115200 |
| PB3 | SPI3_SCK | ATK 光流 SCK | J5 | — |
| PB4 | SPI3_MISO | ATK 光流 MISO | J5 | — |
| PB5 | SPI3_MOSI | ATK 光流 MOSI | J5 | — |
| PC5 | GPIO | ATK 光流 CS | J5 | — |
| PB8 | I2C1_SCL | 罗盘 + OLED (共享) | J6+J14 | 400kHz |
| PB9 | I2C1_SDA | 罗盘 + OLED (共享) | J6+J14 | 400kHz |

#### 通信

| 引脚 | 外设 | 设备 | 插座 | 核心 |
|------|------|------|------|------|
| PB6 | USART1_TX (REMAP) | Debug | J9 | V3F |
| PC6 | USART6_TX | GPS | J6 | V5F |
| PC7 | USART6_RX | GPS | J6 | V5F |
| PE7 | USART7_RX | ATK LoRa 数传 | J7 | V5F |
| PE8 | USART7_TX | ATK LoRa 数传 | J7 | V5F |
| PE0 | USART8_RX | ESP8266 WiFi | J8 | V5F |
| PE1 | USART8_TX | ESP8266 WiFi | J8 | V5F |

#### 存储 / 烧录

| 引脚 | 外设 | 设备 | 插座 |
|------|------|------|------|
| PB12 | GPIO (CS) | SD 卡 CS | J13 |
| PB13 | SPI2_SCK | SD 卡 SCK | J13 |
| PB14 | SPI2_MISO | SD 卡 MISO | J13 |
| PB15 | SPI2_MOSI | SD 卡 MOSI | J13 |
| PA13 | SWD_SWDIO | 烧录 | J12 |
| PA14 | SWD_SWCLK | 烧录 | J12 |

### 2.3 串口分配总览

| USART | TX | RX | 设备 | 核心 | 备注 |
|-------|----|----|------|------|------|
| USART1 | PB6 | — | Debug | V3F | REMAP |
| USART2 | — | PA3 | IBUS 接收机 | V3F | 115200, 仅 RX |
| USART3 | — | PB11 | ATK IMU | V3F | 460800 bps |
| USART4 | — | PC11 | TFmini 下视 | V3F | 115200 |
| USART5 | — | PD2 | TFmini 前向 | V5F | 115200 |
| USART6 | PC6 | PC7 | GPS+罗盘 | V5F | 115200 |
| USART7 | PE8 | PE7 | ATK LoRa | V5F | 115200 |
| USART8 | PE1 | PE0 | ESP8266 WiFi | V5F | 115200 |

### 2.4 SPI 分配

| SPI | 引脚 | 设备 |
|-----|------|------|
| SPI2 | PB12/13/14/15 | SD 卡 |
| SPI3 | PB3/4/5 + PC5 | ATK 光流 |

### 2.5 I2C 分配

| I2C | SCL | SDA | 设备 | 速率 |
|-----|-----|-----|------|------|
| I2C1 | PB8 | PB9 | 罗盘 QMC5883L (0x0D) + OLED SSD1306 (0x3C) | 400kHz |

### 2.6 ADC 分配

| 通道 | 引脚 | 检测项 | 电路 |
|------|------|--------|------|
| ADC1_IN1 | PA1 | 电池电压 | 电阻分压 10k:2k (6:1) |
| ADC1_IN2 | PA2 | 电池电流 | ACS712-30A 模块 + 分压 |

### 2.7 空闲接口

| 接口 | 数量 | 说明 |
|------|------|------|
| SPI1 (PA4/5/6/7) | 1 路 | 引出，可扩展 |
| I2C2 | 1 路 | 引出，可扩展 |
| PA0 / PC0-4 等 | 若干 | 空闲 GPIO，引出 |
| CAN1 / CAN2 / CAN3 | 3 路 | 引出 |
| USB HS | 1 路 | 有线 QGC |

---

## 三、接插件清单（全 XH2.54）

| 编号 | 设备 | 规格 | 脚数 | 线序 |
|------|------|------|------|------|
| J1 | IA6B 接收机 | XH2.54-3P | 3 | GND / 5V / IBUS |
| J2 | ATK 十轴 IMU | XH2.54-4P | 4 | GND / 3.3V / TX / — |
| J3 | TFmini 下视 TOF | XH2.54-4P | 4 | GND / 5V / TX / — |
| J4 | TFmini 前向 TOF | XH2.54-4P | 4 | GND / 5V / TX / — |
| J5 | ATK 光流 | XH2.54-6P | 6 | GND / 3.3V / SCK / MISO / MOSI / CS |
| J6 | GPS+罗盘 | XH2.54-6P | 6 | GND / 5V / TX / RX / SDA / SCL |
| J7 | ATK LoRa 数传 | XH2.54-4P | 4 | GND / 3.3V / TX / RX |
| J8 | ESP8266 WiFi | XH2.54-4P | 4 | GND / 3.3V / TX / RX |
| J9 | Debug 串口 | XH2.54-4P | 4 | GND / 3.3V / TX / — |
| J10 | ESC ×4 | XH2.54-6P | 6 | GND / S1 / S2 / S3 / S4 / — |
| J11 | 蜂鸣器 | XH2.54-2P | 2 | GND / SIG |
| J12 | SWD 烧录 | XH2.54-4P | 4 | GND / 3.3V / SWDIO / SWCLK |
| J13 | SD 卡 | MicroSD 卡座 | — | 板载贴片 |
| J14 | OLED 小屏 | XH2.54-4P | 4 | GND / 3.3V / SDA / SCL |

---

## 四、电源设计

### 4.1 电源树

```
4S 电池 (14.8~16.8V)
  │
  ├──→ MP1584 开关降压 → 5V / 3A
  │     │
  │     ├── J1  → IA6B 接收机 (5V, ~100mA)
  │     ├── J3  → TFmini 下视 (5V, ~120mA)
  │     ├── J4  → TFmini 前向 (5V, ~120mA)
  │     ├── J6  → GPS (5V, ~50mA)
  │     └── J11 → 蜂鸣器 (5V, ~80mA)
  │     │
  │     └──→ AMS1117-3.3 LDO → 3.3V / 1A
  │           │
  │           ├── CH32H417 (3.3V, ~150mA)
  │           ├── J2  → ATK IMU (3.3V, ~50mA)
  │           ├── J5  → ATK 光流 (3.3V, ~50mA)
  │           ├── J7  → ATK LoRa (3.3V, ~120mA)
  │           ├── J8  → ESP8266 (3.3V, ~300mA)
  │           └── J13 → SD 卡 (3.3V, ~100mA)
  │
  ├──→ 电阻分压 → PA1 (ADC, 电池电压)
  └──→ ACS712 模块 5V 供电, VOUT 分压 → PA2 (ADC, 电池电流)
```

### 4.2 功率估算

| 电压轨 | 峰值电流 | 设计余量 |
|--------|---------|---------|
| 5V | ~470mA | **3A** (MP1584) |
| 3.3V | ~770mA | **1A** (AMS1117) |

### 4.3 电池保护阈值

| 等级 | 电压 (4S) | 动作 |
|------|----------|------|
| 满电 | 16.8V | — |
| 低电预警 | 14.8V (3.7V/cell) | LED 闪烁 + MAVLink 告警 |
| 强制降落 | 14.0V (3.5V/cell) | 自动 RTL 返航降落 |

---

## 五、软件架构

### 5.1 双核分工

| | V3F (120MHz, 裸机) | V5F (480MHz, FreeRTOS) |
|---|---|---|
| 角色 | 硬件 I/O 桥 + 安全看门狗 | 飞控算法 + 应用 |
| 传感器 | IBUS / IMU / TOF下视 / 光流(SPI) / 电池ADC / 罗盘I2C | TOF前向 / GPS |
| 通信 | Debug | LoRa / WiFi / MAVLink |
| 存储 | — | SD 卡日志 |
| 控制 | — | 姿态解算 + PID + 混控 + 导航 + 避障 |
| 输出 | TIM1 4路 PWM (从 IPC 读取) | — |
| 保护 | V5F 超时 500ms 强制停转 | — |

### 5.2 中断 / 任务分配

#### V3F 中断

| 中断 | 频率 | 功能 |
|------|------|------|
| USART2 RX | 115200 | IA6B IBUS 14 通道解码 → IPC |
| USART3 RX | 460800 | ATK IMU 姿态帧解析 → IPC |
| USART4 RX | 115200 | TFmini 下视距离 → IPC |
| SPI3 DMA | 100Hz | ATK 光流 Δx/Δy → IPC |
| ADC1 DMA | 10Hz | 电池电压/电流 → IPC |
| I2C1 轮询 | 50Hz | QMC5883L 罗盘读取 → IPC |
| TIM1 | 50Hz | IPC motor_pwm → CCR 更新 |
| SysTick | 1kHz | 喂狗 + V5F 心跳超时检测 |

#### V5F 任务 (FreeRTOS)

| 任务 | 频率 | 优先级 | 功能 |
|------|------|--------|------|
| vTaskFusion | 1kHz | 最高 | Madgwick AHRS 姿态解算 |
| vTaskControl | 1kHz | 最高 | 角速率 PID + 姿态 PID + 混控 → motor_pwm |
| vTaskAltitude | 100Hz | 高 | TOF/气压高度融合 + 定高 PID |
| vTaskPosition | 50Hz | 高 | GPS/光流位置 + 速度 PID |
| vTaskAvoidance | 50Hz | 高 | 前向 TOF 避障检测 |
| vTaskNavigation | 10Hz | 中 | 航点管理 + L1 路径跟随 |
| vTaskTelemetry | 10Hz | 中 | MAVLink 遥测发送 (LoRa + WiFi) |
| vTaskCompass | 50Hz | 高 | I2C1 罗盘读取 + 航向计算 |
| vTaskGPS | 10Hz | 中 | GPS NMEA 解析 |
| vTaskCommand | 5Hz | 低 | 地面站指令处理 |
| vTaskLogger | 5Hz | 低 | SD 卡飞行数据记录 |

### 5.3 核间通信 (IPC)

```c
// V3F → V5F (传感器数据)
typedef struct {
    // ATK IMU (USART3, 200Hz)
    float    euler_roll;
    float    euler_pitch;
    float    euler_yaw;
    float    gyro_x, gyro_y, gyro_z;     // 角速率 rad/s
    float    accel_x, accel_y, accel_z;   // 加速度 m/s^2
    float    baro_altitude;               // 气压高度 m
    float    temperature;                 // IMU 温度
    uint32_t imu_timestamp_ms;

    // TFmini 下视 TOF (USART4, 100Hz)
    float    tof_altitude;                // 高度 m
    uint8_t  tof_updated;

    // ATK 光流 (SPI3, 100Hz, 高度来自 TFmini 下视)
    float    flow_dx, flow_dy;            // 位移 cm (已用 TFmini 高度换算)
    float    flow_vel_x, flow_vel_y;      // 速度 cm/s
    uint8_t  flow_quality;                // squal 表面质量 0~255
    uint8_t  flow_updated;

    // IA6B IBUS (USART2, 115200, ~140Hz)
    uint16_t rc_channels[14];            // IBUS 最大 14 通道, 1000~2000
    uint8_t  rc_updated;

    // 电池 (ADC1, 10Hz)
    float    battery_voltage;
    float    battery_current;
    float    battery_consumed_mah;

    // GPS 罗盘 (I2C1, 50Hz)
    int16_t  mag_x, mag_y, mag_z;         // 磁力计原始值
    float    heading;                      // 罗盘航向 °

    // 心跳
    uint32_t v3f_heartbeat_ms;
} v3f_to_v5f_t;

// V5F → V3F (控制输出)
typedef struct {
    uint16_t motor_pwm[4];                // 1000~2000 μs
    uint8_t  motor_updated;
    uint8_t  arm_state;                   // 0=锁定 1=解锁
    uint16_t buzzer_freq;                 // 0=关闭
    uint32_t v5f_heartbeat_ms;
} v5f_to_v3f_t;
```

### 5.4 控制回路架构

```
位置控制 (V5F, 50Hz)
  GPS 水平位置 → [P] → 目标速度
  光流位移     → [P] → 目标速度 (室内)
                      │
速度控制 (V5F, 50Hz)   │
  目标速度 → [PID] → 目标角度
                      │
姿态控制 (V5F, 500Hz)  │
  目标角度 → [PID] → 目标角速率
                      │
角速率控制 (V5F, 1kHz) │
  目标角速率 → [PID] → 力矩
                      │
混控器 (V5F, 1kHz)     │
  力矩 + 油门 → motor[0..3]
                      │
            ┌─────────┘
            ▼
         IPC → V3F → TIM1 CCR → ESC ×4
```

### 5.5 定高控制

```c
// 高度融合
altitude = TOF * tof_weight + baro * (1 - tof_weight);
// TOF < 4m: tof_weight = 1.0  (TOF 主导)
// TOF > 4m: tof_weight = 0.0  (气压主导)
// 4~6m: 线性过渡

// 垂直速度 (加速度计 Z 轴积分 + 高度微分)
vz = 0.95 * (vz + az * dt) + 0.05 * (altitude_diff / dt);

// 定高 PID
alt_error = target_alt - altitude;
vz_target = altitude_pid(alt_error);     // 高度环 → 目标垂直速度
thrust_adj = vz_pid(vz_target - vz);     // 速度环 → 油门增量
```

### 5.6 X 型混控矩阵

```c
// 输入: roll_cmd, pitch_cmd, yaw_cmd (归一化 ±1), thrust (0~1)
// 输出: motor[0..3] (0~1)

motor[0] = thrust + roll_cmd + pitch_cmd - yaw_cmd;  // 前右 CW
motor[1] = thrust - roll_cmd - pitch_cmd - yaw_cmd;  // 后左 CW
motor[2] = thrust - roll_cmd + pitch_cmd + yaw_cmd;  // 前左 CCW
motor[3] = thrust + roll_cmd - pitch_cmd + yaw_cmd;  // 后右 CCW

// 限幅到 [0, 1]
// 映射到 PWM 脉宽
pwm_us[i] = 1000 + motor[i] * 1000;  // 1000~2000 μs
```

---

## 六、飞行模式

### 6.1 模式定义

| 模式 | CH5 值 | 定高 | 定位 | 行为 |
|------|--------|------|------|------|
| STABILIZE | <1200 | ❌ | ❌ | 自稳，手动驾驶 |
| ALTHOLD | 1200~1800 | ✅ | ❌ | 高度锁定，手动位置 |
| AUTO | >1800 | ✅ | ✅ | 航点巡航，全自主 |
| RTL | 失控触发 | ✅ | ✅ | 自动返航降落 |
| LAND | 失控/GCS | ✅ | — | 原地降落 |

### 6.2 控制权仲裁

```
优先级: 遥控器 > 地面站 > 飞控自主

遥控器断线 >1s → GPS 有效 → RTL
              → GPS 无效 → LAND
AUTO 模式下摇杆偏离 >10% → 自动退到 ALTHOLD
CH6 (SWA): >1500=ARM解锁 <1200=DISARM锁定
```

### 6.3 前向避障 (所有模式生效)

```c
float dist = tfmini_forward_read();

if (dist < 0.8f) {
    强制悬停 (HOLD);           // 等待接管或重规划
} else if (dist < 2.0f) {
    speed_limit = (dist - 0.8f) / 1.2f;  // 渐进减速
} else {
    speed_limit = 1.0f;        // 全速
}
```

---

## 七、通信协议

### 7.1 ESC PWM 信号

| 项目 | 参数 |
|------|------|
| 频率 | 50Hz (20ms 周期) |
| 脉宽范围 | 1000~2000 μs |
| 停转 | 1000 μs |
| 怠速 | 1060 μs |
| 全速 | 2000 μs |
| 死区 | 30 (0.21μs @ 144MHz TIM1) |

### 7.2 IBUS 接收机协议

| 项目 | 参数 |
|------|------|
| 接口 | USART2, 仅 RX |
| 波特率 | 115200, 8N1 |
| 帧长 | 32 字节 |
| 帧率 | ~140Hz (每帧 ~7ms) |
| 通道数 | 14 通道 (CH0~CH13) |
| 取值范围 | 1000~2000 (同 PWM 脉宽) |

**帧格式:**

```
Byte  0    1    2    3    4    5   ...  29   30   31
    0x20 0x40 CH0L CH0H CH1L CH1H ... CH13H CKL CKH
    │    │    └────── 28 字节通道数据 ──────┘  └─ 校验 ─┘
    │    └── 帧头高字节
    └── 帧头低字节
```

**校验和:** `checksum = 0xFFFF - (前 30 字节累加和)`

**CH5 模式开关 (FlySky 3 段):**
```
CH5 < 1200  → STABILIZE (自稳)
CH5 1200~1800 → ALTHOLD (定高)
CH5 > 1800  → AUTO (航点)
```

**CH6 解锁 (FlySky SWA):**
```
CH6 > 1500 → ARM 解锁
CH6 < 1200 → DISARM 锁定
```

**解析流程:**
```c
// USART2 RX 中断, 状态机解析
// 1. 找帧头 0x20 0x40
// 2. 收 32 字节
// 3. 校验 checksum
// 4. 提取 14 通道 → rc_channels[0..13]
```

### 7.3 ATK 十轴 IMU

| 项目 | 参数 |
|------|------|
| 接口 | USART3, 仅 RX |
| 波特率 | 460800, 8N1 |
| 帧率 | 200Hz |
| 输出 | 欧拉角 + 角速率 + 加速度 + 气压 + 温度 |
| 帧头 | 0x55 |

### 7.4 TFmini Plus (TOF)

| 项目 | 下视 (J3) | 前向 (J4) |
|------|----------|----------|
| 接口 | USART4 | USART5 |
| 波特率 | 115200 | 115200 |
| 帧率 | 100Hz (默认) | 100Hz |
| 输出 | 标准串口 ASCII 距离值 |

### 7.5 ATK 光流 (PMW3901)

| 项目 | 参数 |
|------|------|
| 接口 | SPI3 (PB3/4/5) + CS (PC5) |
| 速率 | 2MHz |
| 帧率 | 100~200Hz |
| 有效高度 | 5cm~4m |
| 高度来源 | TFmini 下视 TOF (USART4), **不使用** VL53L0X |

**SPI 突发读取 (Motion Burst, 12 字节):**

```
Byte  0        1           2     3     4     5     6     7     8        9        10       11
     motion  observation  ΔX_L  ΔX_H  ΔY_L  ΔY_H  squal rawSum  maxRaw  minRaw  shutter_L shutter_H
```

| 字段 | 说明 |
|------|------|
| motion | bit0=运动发生, bit3=帧数据就绪 |
| ΔX, ΔY | 像素位移 (int16, 小端) |
| squal | 表面质量, <10 丢弃该帧 |
| shutter | 自动快门值 |

**像素 → 位移转换 (高度来自 TFmini):**

```c
#define RESOLUTION  0.213f    // 1m 高度下 1 像素对应的位移 (cm), 需标定

float height_m = tof_altitude;                 // TFmini 下视高度 (USART4)
float coeff    = RESOLUTION * height_m;        // 像素→cm 系数

// 倾角补偿: 消除机体倾斜带来的像素误差
float comp_x = 480.0f * tanf(pitch_rad);
float comp_y = 480.0f * tanf(roll_rad);

float dx_cm = coeff * (burst.deltaX + comp_x);
float dy_cm = coeff * (burst.deltaY + comp_y);
float vx = dx_cm / dt;                         // cm/s
```

**注意事项:**
- 传感器朝下，图像运动与机体运动方向**相反**（代码取反）
- maxRawData/minRawData 连续 1s 为 0 → 传感器故障
- squal < 10 → 数据不可信
- 高度 > 4m 时光流无效应，自动切换纯 GPS/气压计模式

### 7.6 GPS + 罗盘

| 项目 | GPS (USART6) | 罗盘 (I2C1) |
|------|-------------|------------|
| 接口 | USART6 TX/RX (PC6/PC7) | I2C1 SCL/SDA (PB8/PB9) |
| 波特率/速率 | 115200, 8N1 | 400kHz (Fast Mode) |
| 协议 | NMEA ($GNGGA, $GNRMC) | I2C 寄存器读写 |
| 帧率 | 10Hz | 50Hz |
| 芯片 | u-blox M8N / ATGM336H | HMC5883L / QMC5883L |

**罗盘 (I2C) 读取流程:**
```c
// QMC5883L 典型初始化 + 读取 (I2C1, 地址 0x0D)
// 1. 写 0x0B ← 0x01   (SET/RESET 周期)
// 2. 写 0x09 ← 0x1D   (200Hz ODR, ±2G, 512 OSR, 连续模式)
// 3. 读 0x00~0x05 (6 字节: X_L, X_H, Y_L, Y_H, Z_L, Z_H)
// 4. heading = atan2f(mag_y, mag_x) * 57.3f + declination
```

### 7.7 MAVLink 遥测 (LoRa + WiFi 双链路)

| 项目 | ATK LoRa | ESP8266 WiFi |
|------|---------|-------------|
| 接口 | USART7 | USART8 |
| 波特率 | 115200 | 115200 |
| 协议 | MAVLink v2 | MAVLink v2 |
| 频率 | 433MHz | 2.4GHz |
| 距离 | ~3km | ~50m |

---

## 八、项目目录结构

```
Drone_FC_CH32H417/
│
├── V3F/                              # 小核工程 (裸机)
│   ├── Small_Core.wvproj
│   └── User/
│       ├── main.c                    # V3F 入口
│       ├── ch32h417_conf.h
│       ├── ch32h417_it.c/.h          # 中断服务
│       ├── system_ch32h417.c/.h
│       ├── sensor_ibus.c/.h          # IBUS 解码 (USART2)
│       ├── sensor_atk_imu.c/.h       # ATK IMU 串口解析
│       ├── sensor_tof_down.c/.h      # TFmini 下视
│       ├── sensor_flow.c/.h          # ATK 光流 SPI
│       ├── bsp_esc_pwm.c/.h          # TIM1 4路 50Hz PWM
│       ├── bsp_adc.c/.h              # 电池 ADC
│       ├── sensor_compass.c/.h       # I2C1 罗盘 (QMC5883L)
│       └── bsp_ipc.c/.h              # IPC 通信
│
├── V5F/                              # 大核工程 (FreeRTOS)
│   ├── Big_Core.wvproj
│   └── User/
│       ├── main.c                    # V5F 入口 + 任务创建
│       ├── ch32h417_conf.h
│       ├── ch32h417_it.c/.h
│       ├── system_ch32h417.c/.h
│       ├── FreeRTOSConfig.h
│       ├── sensor_tof_forward.c/.h   # TFmini 前向
│       ├── sensor_gps.c/.h           # GPS NMEA 解析
│       ├── comm_lora.c/.h            # LoRa MAVLink
│       ├── comm_wifi.c/.h            # WiFi MAVLink
│       └── bsp_sd_logger.c/.h        # SD 卡日志
│
├── Common/                           # 双核共享
│   ├── ipc_data.h                    # 核间通信数据结构
│   └── flight_defs.h                 # 飞行模式/公共宏
│
├── FC_Core/                          # 飞控算法 (V5F)
│   ├── math/
│   │   ├── quaternion.c/.h           # 四元数运算
│   │   ├── vector3.c/.h              # 三维向量
│   │   └── filters.c/.h              # 低通/互补滤波器
│   ├── attitude/
│   │   ├── madgwick_ahrs.c/.h        # Madgwick AHRS
│   │   └── calibration.c/.h          # 传感器校准
│   ├── control/
│   │   ├── pid.c/.h                  # 通用 PID
│   │   ├── rate_controller.c/.h      # 角速率控制 (1kHz)
│   │   ├── attitude_controller.c/.h  # 姿态控制 (500Hz)
│   │   ├── velocity_controller.c/.h  # 速度控制 (50Hz)
│   │   ├── position_controller.c/.h  # 位置控制 (50Hz)
│   │   ├── altitude_controller.c/.h  # 定高控制 (100Hz)
│   │   └── mixer.c/.h               # X 型混控
│   ├── navigation/
│   │   ├── waypoint_manager.c/.h     # 航点管理
│   │   ├── l1_follower.c/.h          # L1 路径跟随
│   │   └── failsafe.c/.h             # 失控保护
│   └── fusion/
│       └── altitude_fusion.c/.h      # TOF + 气压高度融合
│
├── Config/
│   ├── params.h                      # PID/滤波器参数
│   ├── mixer_config.h                # 机型配置
│   └── waypoints_default.h           # 预设航点
│
├── Middleware/
│   ├── FreeRTOS/                     # RTOS 内核
│   ├── FATFS/                        # 文件系统
│   └── MAVLink/                      # MAVLink v2
│
└── SRC/                              # CH32H417 官方库
    ├── Core/
    ├── Debug/
    ├── Ld/
    ├── Startup/
    └── Peripheral/
```

---

## 九、BOM 清单

### 9.1 PCB 板载元件

| 编号 | 元件 | 型号/值 | 封装 | 数量 |
|------|------|---------|------|------|
| U1 | 飞控 MCU | CH32H417 | LQFP-64 | 1 |
| U2 | 开关降压 | MP1584 | SOP-8 | 1 |
| U3 | LDO | AMS1117-3.3 | SOT-223 | 1 |
| L1 | 电感 | 22μH/3A | CD54 | 1 |
| C1 | 电解电容 | 100μF/25V | Φ6.3 直插 | 1 |
| C2-3 | 钽电容 | 10μF/16V | 1206 | 2 |
| C4-15 | 陶瓷电容 | 0.1μF | 0603 | 12 |
| C16-17 | 陶瓷电容 | 10pF | 0603 | 2 |
| R1 | 电阻 | 10kΩ | 0603 | 1 |
| R2 | 电阻 | 2kΩ | 0603 | 1 |
| R3-6 | 电阻 | 100Ω | 0603 | 4 |
| R7-8 | 电阻 | 4.7kΩ | 0603 | 2 |
| D1 | TVS | SMAJ5.0A | SMA | 1 |
| D2 | LED | 红色 | 0603 | 1 |
| Y1 | 晶振 | 25MHz/10ppm | 3225 | 1 |
| J13 | MicroSD 卡座 | 推拉式 | 贴片 | 1 |

### 9.2 接插件

| 编号 | 规格 | 数量 |
|------|------|------|
| XH2.54-2P 弯针座 | 2P | 1 |
| XH2.54-3P 弯针座 | 3P | 1 |
| XH2.54-4P 弯针座 | 4P | 10 |
| XH2.54-6P 弯针座 | 6P | 2 |
| XH2.54-4P 直针座 | 4P | 1 |
| XT60 母座 | 焊接 | 1 |

### 9.3 外挂模块

| 模块 | 型号 | 数量 |
|------|------|------|
| ESC | CH32V203 六步换相 | 4 |
| 电机 | 2216 1400KV | 4 |
| 桨 | 9443 两叶 | 4 (2CW+2CCW) |
| 电池 | 4S 5200mAh 30C+ | 1 |
| 机架 | F450 | 1 |
| 遥控器 | Flysky i6X + IA6B/IA10B (IBUS) | 1 |
| IMU | ATK 十轴串口 | 1 |
| 下视 TOF | TFmini Plus | 1 |
| 前向 TOF | TFmini Plus | 1 |
| 光流 | ATK 4m SPI | 1 |
| GPS | 串口 GPS + I2C 罗盘 (M8N+QMC5883L) | 1 |
| 数传 | ATK LoRa 串口 | 1 |
| WiFi | ESP8266 | 1 |
| 蜂鸣器 | 有源 5V | 1 |
| OLED | 0.96 寸 SSD1306 (I2C, 0x3C) | 1 |
| 电池检测 | ACS712-30A 模块 | 1 |

### 9.4 线材 / 工具

| 物料 | 规格 | 数量 |
|------|------|------|
| XH2.54 公头 | 2P/3P/4P/6P | 各若干 |
| XH2.54 端子 | — | ~100 |
| 排线 | 26AWG | 若干 |
| 压线钳 | XH2.54 | 1 |
| 热缩管 | 各规格 | 若干 |
| 扎带 | 3×100mm | 若干 |
| 尼龙柱 | M3×6mm | 8 |
| 螺丝 | M3×8mm | 8 |

### 9.5 PCB 规格

| 项目 | 参数 |
|------|------|
| 尺寸 | 45×45mm |
| 层数 | 2 层 |
| 板厚 | 1.6mm |
| 铜厚 | 1oz |
| 安装孔 | M3×4, 间距 30.5×30.5mm |

---

## 十、PCB 插座布局图

```
          ┌──────────────────────────┐
          │  J12 SWD (4P 直针)        │
          │                          │
   J4 ──  │  前向TOF           J1 ──  │
          │                          │
   J6 ──  │  GPS                 J5 ──│
          │                          │
   J2 ──  │  IMU                     │
          │           ┌──────┐       │── J10 ESC×4
   J7 ──  │   LoRa    │ MCU  │       │
          │           │      │       │── J7 LoRa
   J8 ──  │   WiFi    └──────┘       │── J8 WiFi
          │                          │
   J3 ──  │  下视TOF           J13   │
          │                   SD 卡座 │
   J9 ──  │  Debug             J11   │── J11 蜂鸣器
          │                   蜂鸣器   │
          │        ┌──┐              │
          │  XT60 ←│电│              │
          │        │源│              │
          │        └──┘              │
          └──────────────────────────┘

   所有弯针插座朝 PCB 边缘出线
   排线沿机架臂走线，扎带固定
```

---

## 十一、开发阶段

| 阶段 | 内容 | 验证方式 |
|------|------|----------|
| P1 | CH32H417 双核启动 + IPC | 双核各自闪烁 LED |
| P2 | ATK IMU + IBUS + TOF + GPS+罗盘 驱动 | 串口输出各传感器数据 |
| P3 | TIM1 4路 PWM 输出 | 逻辑分析仪验证波形 |
| P4 | 遥控器 → 混控 → PWM | 推杆 → 4 路 PWM 跟随 |
| P5 | Madgwick 姿态解算 + 角速率/姿态 PID | 测试架扰动恢复 |
| P6 | TOF 定高 + GPS/罗盘/光流位置控制 | 系留绳悬停测试 |
| P7 | 航点规划 + L1 跟随 | 方形航线自主飞行 |
| P8 | 避障 + 失控保护 + MAVLink 遥测 | 全功能飞行演示 |
