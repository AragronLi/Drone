# ESC PWM 驱动模块文档

> **串口分配最终确认（2026-05-31）**
> 本项目硬件分配最终以此为准：USART1=Debug(PB6/PB7)、USART2=接收机(PA3)、USART3=陀螺仪(PB10/PB11)、USART4=GPS(PC6/PC7)、USART5=PF5/PE0空闲、USART6=下视激光(PC10/PC11)、USART7=前视激光(PC12/PD2)、USART8=数传(PE8/PE7)、I2C3=罗盘(PA14=SCL/PA13=SDA)。


## 一、模块概述

| 项目 | 说明 |
|------|------|
| 模块名称 | ESC PWM 驱动 |
| 外设 | TIM1 高级定时器 |
| 输出引脚 | PA8 (CH1), PA9 (CH2), PA10 (CH3), PA11 (CH4) |
| 信号标准 | 50Hz PWM, 1000~2000μs 脉宽 |
| 分辨率 | 1μs |
| 极性 | 高电平有效 |
| 用途 | 驱动 4 路无刷电机 ESC（CH32V203 六步换相） |

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

## 二、硬件接线

```
CH32H417 飞控板                  ESC ×4
┌──────────────┐            ┌──────────────┐
│ PA8  TIM1_CH1 ├────────────┤ ESC 1 (前右 CW)  │  J10-2
│ PA9  TIM1_CH2 ├────────────┤ ESC 2 (后左 CW)  │  J10-3
│ PA10 TIM1_CH3 ├────────────┤ ESC 3 (前左 CCW) │  J10-4
│ PA11 TIM1_CH4 ├────────────┤ ESC 4 (后右 CCW) │  J10-5
│ GND           ├────────────┤ GND (共地)       │  J10-1
└──────────────┘            └──────────────┘
```

### 引脚对应

| MCU 引脚 | 外设 | AF | 功能 | 插座 |
|----------|------|-----|------|------|
| PA8 | TIM1_CH1 | AF1 | ESC 1 PWM (前右, CW) | J10-2 |
| PA9 | TIM1_CH2 | AF1 | ESC 2 PWM (后左, CW) | J10-3 |
| PA10 | TIM1_CH3 | AF1 | ESC 3 PWM (前左, CCW) | J10-4 |
| PA11 | TIM1_CH4 | AF1 | ESC 4 PWM (后右, CCW) | J10-5 |

> **引脚说明**: PA9 同时支持 TIM1_CH2(AF1) 和 USART1_TX(AF7)。Debug 串口已移至 PB6(AF7)，PA9 完全释放给 TIM1_CH2。详见 CH32H417 DS 第 2 章引脚定义表。

---

## 三、PWM 信号规格

| 参数 | 值 | 说明 |
|------|------|------|
| 频率 | 50Hz | 20ms 周期, 标准 ESC 协议 |
| 停转脉宽 | 1000μs | 电机完全停止 |
| 怠速脉宽 | 1060μs | 电机最低稳定转速 |
| 全速脉宽 | 2000μs | 电机最大转速 |
| 死区 | — | 1μs 分辨率, 远小于 ESC 死区 (~30μs) |

### 信号示意图

```
20ms ────────────────────────────────────────
│                                            │
│  ┌─────┐                                   │
│  │脉冲 │                                   │
│  │1~2ms│                                   │
└──┘     └───────────────────────────────────┘

停转: 1.0ms ┌──┐____________________________
怠速: 1.06ms ┌───┐___________________________
半速: 1.5ms ┌─────┐_________________________
全速: 2.0ms ┌────────┐______________________
```

---

## 四、TIM1 配置

### 4.1 时钟树

```
SYSCLK (400MHz)
  └── HPRE DIV1 → V5F Core (400MHz)
        └── FPRE DIV4 → V3F Core / HCLK (100MHz)
              └── HB2 → TIM1_CLK = 100MHz
```

### 4.2 定时器参数

| 参数 | 计算公式 | 值 |
|------|---------|-----|
| TIM1_CLK | HCLK | 100 MHz |
| PSC | TIM1_CLK / 1MHz - 1 | 99 |
| TIM1_Tick | TIM1_CLK / (PSC + 1) | 1 MHz (1μs) |
| ARR | 20ms / 1μs - 1 | 19999 |
| PWM 频率 | TIM1_Tick / (ARR + 1) | 50 Hz |

### 4.3 输出模式

| 参数 | 值 | 说明 |
|------|------|------|
| TIM_OCMode | PWM1 | CNT < CCR 时输出 HIGH, CNT >= CCR 输出 LOW |
| TIM_OCPolarity | High | 高电平为有效脉宽 |
| TIM_OCPreload | Enable | CCR 影子寄存器, 更新事件时生效 |

### 4.4 初始化流程

```
RCC 时钟使能 → TIM_DeInit → TIM_InternalClockConfig
  → GPIO AF 配置
  → TIM_TimeBaseInit (PSC=99, ARR=19999)
  → TIM_OC1-4Init (PWM1, CCR=1000)
  → TIM_BDTRConfig (OSSR=Enable)
  → TIM_ARRPreloadConfig / TIM_OCPreloadConfig
  → TIM_CtrlPWMOutputs (MOE=1) / TIM_Cmd (CEN=1)
  → TIM_GenerateEvent (UG=1, 强制装载影子寄存器)
```

---

## 五、软件架构

### 5.1 文件结构

```
USART_Interrupt/
├── Common/
│   ├── bsp_esc_pwm.h         # 驱动头文件 (API 声明、参数宏)
│   └── bsp_esc_pwm.c         # 驱动实现 (TIM1 初始化、PWM 设置)
├── V3F/
│   └── User/
│       └── main.c            # V3F 入口 (调用 Hardware())
└── Common/
    └── hardware.c            # 应用层 (PWM 测试序列)
```

### 5.2 模块分层

```
┌─────────────────────────────┐
│   hardware.c (应用层)        │  ← 调用 bsp_esc_pwm_set / set_all
├─────────────────────────────┤
│   bsp_esc_pwm.c (驱动层)     │  ← TIM1 初始化、CCR 更新
├─────────────────────────────┤
│   ch32h417 硬件抽象层        │  ← RCC / GPIO / TIM1 / AFIO
└─────────────────────────────┘
```

### 5.3 数据流（双核架构）

```
V5F (FreeRTOS)                     V3F (裸机)
┌──────────────────┐              ┌──────────────────┐
│ vTaskControl     │              │                  │
│  PID → motor[4]  │──IPC_CH1──→ │ g_esc_pwm[4]     │
│  (0.0 ~ 1.0)     │  映射到μs   │ ↓                │
│                  │              │ TIM_SetComparex  │
│                  │              │ ↓                │
│                  │              │ TIM1 → PA8~11    │
│                  │              │ ↓                │
│                  │              │ ESC ×4           │
└──────────────────┘              └──────────────────┘

当前阶段: V3F 独立测试, g_esc_pwm[4] 由测试序列直接写入
```

---

## 六、API 参考

### 6.1 初始化

```c
void bsp_esc_pwm_init(void);
```

初始化 TIM1 四路 PWM 输出，所有通道初始脉宽 = 1000μs（停转）。

- 时钟: RCC_HB2Periph_GPIOA | RCC_HB2Periph_TIM1 | RCC_HB2Periph_AFIO
- GPIO: PA8~PA11, AF1, 推挽复用
- 调用 TIM_DeInit 复位, TIM_InternalClockConfig 选择内部时钟
- BDTR OSSR=Enable 确保运行态输出受 OCx 控制
- 初始化后打印 HCLKClock 和 TIM1 寄存器值用于验证

### 6.2 设置单路 PWM

```c
void bsp_esc_pwm_set(uint8_t motor, uint16_t pulse_us);
```

| 参数 | 取值 | 说明 |
|------|------|------|
| motor | ESC_MOTOR1 ~ ESC_MOTOR4 | 电机编号 (0~3) |
| pulse_us | 1000 ~ 2000 | 脉宽 μs, 自动钳位 |

### 6.3 设置四路 PWM

```c
void bsp_esc_pwm_set_all(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
```

一次性更新四路脉宽，值自动写入 TIM1 CCR1~4。

### 6.4 预定义宏

```c
#define ESC_MOTOR1     0
#define ESC_MOTOR2     1
#define ESC_MOTOR3     2
#define ESC_MOTOR4     3

#define ESC_PWM_MIN_US   1000   // 停转
#define ESC_PWM_IDLE_US  1060   // 怠速
#define ESC_PWM_MAX_US   2000   // 全速
```

### 6.5 全局变量

```c
volatile uint16_t g_esc_pwm[4];  // 四路当前脉宽, 调试器可观察
```

---

## 七、混控映射（X 型四旋翼）

```
        前
    M3 (CCW)    M1 (CW)
        \       /
         机体中心
        /       \
    M2 (CW)    M4 (CCW)
        后

混控公式 (V5F 姿态控制输出):
  motor[0] = thrust + roll + pitch - yaw    (M1 前右)
  motor[1] = thrust - roll - pitch - yaw    (M2 后左)
  motor[2] = thrust - roll + pitch + yaw    (M3 前左)
  motor[3] = thrust + roll - pitch + yaw    (M4 后右)

  thrust: 0.0~1.0  (0=停转, 1=全速)
  roll/pitch/yaw: -0.5~+0.5 (归一化力矩)

PWM 映射:
  pwm_us[i] = 1000 + motor[i] * 1000
```

---

## 八、调试输出示例

初始化成功后，Debug 串口 (USART1, PB6) 输出:

```
V3F 硬件初始化
[IMU] 陀螺仪量程: 500 dps
[IMU] 加速度计量程: 4 G
[IMU] 初始化完成, 波特率=115200
[IMU] 开始读取数据...
[ESC] HCLKClock=100000000 Hz
[ESC] TIM1 PSC=99 ARR=19999 CCR1=1000 CCR2=1000 CCR3=1000 CCR4=1000
[ESC] PWM 初始化完成, 四路统一输出 1060us
姿态: R=1.23 P=-0.45 Y=87.60
陀螺:    0.12   -0.08   15.67 dps  加计:  0.001  0.005 -1.004 G
...
```

---

## 九、PWM 测试

当前测试阶段：四路统一输出 1060μs（怠速），固定不变。

```c
// hardware.c 中:
bsp_esc_pwm_set_all(ESC_PWM_IDLE_US, ESC_PWM_IDLE_US,
                    ESC_PWM_IDLE_US, ESC_PWM_IDLE_US);  // 1060, 1060, 1060, 1060
```

逻辑分析仪测量四路 PA8/PA9/PA10/PA11，应看到一致的 50Hz / 1060μs 波形。

> **注意**: 测试时 ESC 务必**不要接电机**，或电机**不要装桨**。

---

## 十、逻辑分析仪验证

### 测量点

| 通道 | 引脚 | 预期信号 |
|------|------|---------|
| CH0 | PA8 | 50Hz, 1000~1200μs 正向脉冲 |
| CH1 | PA9 | 同上, 与 CH0 相位差取决于测试序列 |
| CH2 | PA10 | 同上 |
| CH3 | PA11 | 同上 |

### 预期波形（相位 1: 全部怠速）

```
PA8 ┌──────┐                          ┌────── 1060μs HIGH
    ┘      └──────────────────────────┘
    │←1060μs→│←──── 18940μs LOW ────→│
    │←────────── 20ms ──────────────→│

PA9/PA10/PA11: 同上
```

---

## 十一、配置参数速查

所有可配置宏定义位于 [bsp_esc_pwm.h](Common/bsp_esc_pwm.h):

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `ESC_PWM_FREQ_HZ` | 50 | PWM 频率 Hz |
| `ESC_PWM_PERIOD_US` | 20000 | PWM 周期 μs |
| `ESC_PWM_MIN_US` | 1000 | 停转脉宽 μs |
| `ESC_PWM_IDLE_US` | 1060 | 怠速脉宽 μs |
| `ESC_PWM_MAX_US` | 2000 | 全速脉宽 μs |

TIM1 参数硬编码于 [bsp_esc_pwm.c](Common/bsp_esc_pwm.c) `bsp_esc_pwm_init()`:

| 参数 | 值 | 说明 |
|------|------|------|
| TIM_Prescaler | 99 | 100MHz / 100 = 1MHz |
| TIM_Period | 19999 | 1MHz / 20000 = 50Hz |

> **修改时钟时**: 若 V3F 核心频率不是 100MHz，需重新计算 PSC 以保证 1MHz tick。

---

## 十二、参考资料

- CH32H417 参考手册 (RM) — TIM1 高级定时器章节
- [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md) — 系统架构 / 引脚分配 / 混控矩阵
- [ESC_PWM_MODULE.md](ESC_PWM_MODULE.md) — 本文档
