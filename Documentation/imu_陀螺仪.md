# ATK-MS901M 十轴 IMU 模块集成文档

## 一、模块概述

| 项目 | 说明 |
|------|------|
| 模块型号 | 正点原子 ATK-MS901M 十轴串口 IMU |
| 传感器 | ICM-20602 (6轴陀螺仪+加速度计) + AK8963C (3轴磁力计) + SPL06-001 (气压计) |
| 通信接口 | UART 串口，自定义二进制协议 |
| 出厂波特率 | 115200 bps (可通过上位机 / 指令修改，最高 921600) |
| 回传频率 | 0.1 ~ 200 Hz 可配置 |
| 输出数据 | 欧拉角、四元数、角速率、加速度、磁场强度、气压/海拔、温度 |

---

## 二、硬件接线

### 飞控插座 J2 (XH2.54-4P)

```
IMU 模块                CH32H417 飞控板 (J2)
┌──────────┐            ┌────────────┐
│ GND      ├────────────┤ GND        │
│ 3.3V     ├────────────┤ 3.3V       │
│ TX       ├────────────┤ PB11 (RX)  │
│ RX       ├────────────┤ PB10 (TX)  │  ← 配置时需要，纯接收可省略
└──────────┘            └────────────┘
```

### 引脚对应

| MCU 引脚 | 外设 | 功能 | 说明 |
|----------|------|------|------|
| PB10 | USART3_TX | → IMU RX | 发送配置指令 (量程、回传速率等) |
| PB11 | USART3_RX | ← IMU TX | 接收传感器数据帧 |

> **注意**: 若 IMU 已通过上位机 (ATK_IMU) 预配置并固化到 Flash，则 TX 线可省略，仅需 RX 接收。

### 串口参数

| 参数 | 值 |
|------|------|
| 波特率 | 115200 (出厂默认) / 460800 (飞控目标) |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 硬件流控 | 无 |

---

## 三、通信协议

### 3.1 帧格式

```
┌────────┬────────┬────────┬────────┬───────────┬──────────┐
│ Byte 0 │ Byte 1 │ Byte 2 │ Byte 3 │ Byte 4..N │ Byte N+1 │
│ 帧头L  │ 帧头H  │  ID    │ 长度   │ 数据字段   │ 校验和    │
│ 0x55   │ 0x55/AF│        │        │           │          │
└────────┴────────┴────────┴────────┴───────────┴──────────┘
```

- **帧头高位 = 0x55**: 上传帧 (模块主动上传)
- **帧头高位 = 0xAF**: 应答帧 (MCU 读写寄存器后的应答)
- **校验和**: 前 N+1 字节累加和取低 8 位

### 3.2 上传帧 ID

| ID | 名称 | 数据内容 | 字节数 |
|----|------|---------|--------|
| 0x01 | 姿态角 | 横滚(2B) + 俯仰(2B) + 偏航(2B) | 6 |
| 0x02 | 四元数 | q0(2B) + q1(2B) + q2(2B) + q3(2B) | 8 |
| 0x03 | 陀螺仪+加速度计 | 加计X(2B) Y(2B) Z(2B) + 陀螺X(2B) Y(2B) Z(2B) | 12 |
| 0x04 | 磁力计 | X(2B) + Y(2B) + Z(2B) + 温度(2B) | 8 |
| 0x05 | 气压计 | 气压(4B) + 海拔(4B) + 温度(2B) | 10 |
| 0x06 | 端口数据 | D0(2B) + D1(2B) + D2(2B) + D3(2B) | 8 |

### 3.3 数据转换公式

| 数据类型 | 转换公式 |
|---------|---------|
| 姿态角 (°) | `float = int16 / 32768 × 180` |
| 四元数 | `float = int16 / 32768` |
| 角速率 (°/s) | `float = int16 / 32768 × 量程` (量程: 250/500/1000/2000 dps) |
| 加速度 (G) | `float = int16 / 32768 × 量程` (量程: 2/4/8/16 G) |
| 温度 (°C) | `float = int16 / 100` |

### 3.4 寄存器读写

**发送读命令:**
```
0x55  0xAF  (id | 0x80)  0x01  0x00  checksum
```

**发送写命令 (1 字节):**
```
0x55  0xAF  id  0x01  data  checksum
```

**发送写命令 (2 字节):**
```
0x55  0xAF  id  0x02  dataL  dataH  checksum
```

### 3.5 常用寄存器 ID

| ID | 名称 | 取值 | 说明 |
|----|------|------|------|
| 0x03 | GYROFSR | 0=250, 1=500, 2=1000, 3=2000 | 陀螺仪量程 (dps) |
| 0x04 | ACCFSR | 0=2, 1=4, 2=8, 3=16 | 加速度计量程 (G) |
| 0x07 | BAUD | 0~8 | 波特率 (0=4800 ~ 8=921600) |
| 0x08 | RETURNSET | Bit0~Bit5 | 回传内容 (位掩码) |
| 0x0A | RETURNRATE | 0~10 | 回传频率 (0=0.1Hz ~ 10=200Hz) |

---

## 四、软件架构

### 4.1 文件结构

```
USART_Interrupt/
├── Common/
│   ├── atk_ms901m.h          # 协议层头文件 (帧ID、数据结构、API 声明)
│   ├── atk_ms901m.c          # 协议解析器 (状态机帧解析、数据提取)
│   ├── atk_ms901m_uart.h     # 串口层头文件 (USART3 引脚定义、波特率)
│   ├── atk_ms901m_uart.c     # 串口驱动 (初始化、中断、FIFO)
│   ├── hardware.h            # 硬件头文件 (含 IMU 头文件引用)
│   └── hardware.c            # V3F: IMU 初始化 + 数据读取主循环
├── V3F/
│   └── User/
│       └── ch32h417_it.c     # 中断向量 (USART3_IRQHandler)
└── V5F/
    └── ...
```

### 4.2 模块分层

```
┌─────────────────────────────┐
│   hardware.c (应用层)        │  ← 调用 atk_ms901m_get_* 获取数据
├─────────────────────────────┤
│   atk_ms901m.c (协议层)      │  ← 帧解析状态机、数据格式转换
├─────────────────────────────┤
│   atk_ms901m_uart.c (驱动层) │  ← USART3 初始化、中断、FIFO
├─────────────────────────────┤
│   ch32h417 硬件抽象层        │  ← RCC / GPIO / NVIC / USART
└─────────────────────────────┘
```

### 4.3 数据流

```
IMU 模块 TX ──USART3_RX──▶ 中断 ISR
                              │
                    读取字节写入 FIFO (256 Bytes 环形缓冲)
                              │
                    主循环轮询 atk_ms901m_get_xxx()
                              │
                    状态机从 FIFO 逐字节解析
                              │
                    完整帧 + 校验通过
                              │
                    提取各字段并转换为物理量
                              │
                    当前: printf 输出 / 将来: IPC → V5F
```

### 4.4 中断配置

| 中断源 | 外设 | 函数 | 说明 |
|--------|------|------|------|
| USART3 RXNE | USART3 | `USART3_IRQHandler()` → `ATK_MS901M_UART_IRQHandler()` | 接收一个字节 → 写入 FIFO |
| USART3 ORE | USART3 | 同上 | 溢出错误 → 读 STATR+DATAR 清除 |

---

## 五、API 参考

### 5.1 初始化

```c
uint8_t atk_ms901m_init(uint32_t baudrate);
```

初始化 USART3、读取模块量程配置、设置回传内容为 姿态 + 陀螺仪/加速度计、回传频率 200Hz。

### 5.2 数据获取

```c
uint8_t atk_ms901m_get_attitude(atk_ms901m_attitude_data_t *dat, uint32_t timeout);
uint8_t atk_ms901m_get_quaternion(atk_ms901m_quaternion_data_t *dat, uint32_t timeout);
uint8_t atk_ms901m_get_gyro_accelerometer(atk_ms901m_gyro_data_t *gyro, atk_ms901m_accelerometer_data_t *accel, uint32_t timeout);
uint8_t atk_ms901m_get_magnetometer(atk_ms901m_magnetometer_data_t *dat, uint32_t timeout);
uint8_t atk_ms901m_get_barometer(atk_ms901m_barometer_data_t *dat, uint32_t timeout);
uint8_t atk_ms901m_get_port(atk_ms901m_port_data_t *dat, uint32_t timeout);
```

每个函数从接收 FIFO 中解析一个完整帧。若 FIFO 中暂无匹配帧则等待，超时返回 `ATK_MS901M_ETIMEOUT`。

- `timeout` 参数单位 ms (近似值，每轮循环约 100us)
- 返回值: `ATK_MS901M_EOK`=0 表示成功

### 5.3 数据结构

```c
typedef struct {
    float roll;     // 横滚角 °
    float pitch;    // 俯仰角 °
    float yaw;      // 偏航角 °
} atk_ms901m_attitude_data_t;

typedef struct {
    struct { int16_t x, y, z; } raw;  // 原始值
    float x, y, z;                     // 角速率 dps
} atk_ms901m_gyro_data_t;

typedef struct {
    struct { int16_t x, y, z; } raw;  // 原始值
    float x, y, z;                     // 加速度 G
} atk_ms901m_accelerometer_data_t;
```

---

## 六、调试输出示例

初始化成功后，Debug 串口 (USART1, PB6) 输出:

```
V3F 硬件初始化
[IMU] 陀螺仪量程: 500 dps
[IMU] 加速度计量程: 4 G
[IMU] 初始化完成, 波特率=115200
[IMU] 开始读取数据...
姿态: R=1.23 P=-0.45 Y=87.60
陀螺:    0.12   -0.08   15.67 dps  加计:  0.001  0.005 -1.004 G
姿态: R=1.25 P=-0.42 Y=87.55
陀螺:    0.15   -0.10   15.50 dps  加计:  0.002  0.003 -1.001 G
...
```

---

## 七、故障排查

| 现象 | 可能原因 | 解决方案 |
|------|---------|---------|
| `初始化失败` | 接线错误或波特率不匹配 | 检查 PB10/PB11 接线，确认模块波特率 |
| 初始化通过但无数据打印 | IMU 模块未上传数据 | 检查 `RETURNSET` 和 `RETURNRATE` 配置是否生效 |
| 数据显示全零 | 量程读取失败使用了默认值 | 检查 IMU 模块是否正常工作 (LED 状态) |
| 数据断断续续 | 波特率过高或 FIFO 溢出 | 增大 `ATK_MS901M_UART_RX_FIFO_SIZE` |
| 编译报 include 错误 | IDE IntelliSense 未配置路径 | 在 MRS 中编译, 或配置 `.vscode/c_cpp_properties.json` |

---

## 八、配置参数速查

所有可配置宏定义位于 [atk_ms901m_uart.h](Common/atk_ms901m_uart.h):

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `ATK_MS901M_UART_BAUDRATE` | 115200 | 串口波特率, 可改为 460800 |
| `ATK_MS901M_UART_RX_FIFO_SIZE` | 256 | 接收缓冲区大小 (字节) |
| `ATK_MS901M_UART` | USART3 | 使用的串口 |
| `ATK_MS901M_UART_TX_PIN` | GPIO_Pin_10 | TX 引脚 (PB10) |
| `ATK_MS901M_UART_RX_PIN` | GPIO_Pin_11 | RX 引脚 (PB11) |

---

## 九、参考资料

- ATK-MS901M 模块用户手册 V1.0
- ICM-20602 数据手册
- AK8963C 数据手册
- SPL06-001 气压传感器数据手册
- D 盘 IMU 源码: `/d/imu/2，程序源码/ATK-MS901M模块测试实验/`
