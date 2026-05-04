# ROV 控制系统 v2.0 重构完整文档

> **项目**：水下机器人（ROV）运动控制系统  
> **MCU**：STM32H750VBT6 (Cortex-M7, 480MHz, 128KB Flash)  
> **RTOS**：RT-Thread 3.1.5  
> **重构日期**：2026-05-04  
> **GitHub**：https://github.com/sanfenrujian/ROV-Control-System

---

## 目录

1. [重构动机](#1-重构动机)
2. [架构总览](#2-架构总览)
3. [I2C 总线安全 — 核心改动](#3-i2c-总线安全--核心改动)
4. [UART 多路管理](#4-uart-多路管理)
5. [统一系统状态](#5-统一系统状态)
6. [统一 PID 控制器](#6-统一-pid-控制器)
7. [运动控制重构](#7-运动控制重构)
8. [线程管理器重构](#8-线程管理器重构)
9. [修复的 Bug 清单](#9-修复的-bug-清单)
10. [文件变更对照表](#10-文件变更对照表)
11. [API 参考](#11-api-参考)
12. [Keil 工程迁移步骤](#12-keil-工程迁移步骤)

---

## 1. 重构动机

原项目存在以下严重问题，影响系统稳定性和可维护性：

| 序号 | 问题 | 严重度 | 影响 |
|:----:|------|:------:|------|
| 1 | 多路 UART 共用同一个 `data` 字节变量 | 🔴 致命 | 中断并发时遥控/传感器数据互相覆盖 |
| 2 | 软件 I2C 通过全局变量动态切换引脚，无锁保护 | 🔴 致命 | MPU6050 和 MS5837 同时操作导致总线冲突 |
| 3 | RT-Thread 堆空间仅 1KB | 🔴 致命 | `malloc` 动态分配耗尽，系统崩溃 |
| 4 | 局部数组 `jiaodu[10000]` 分配在栈上 | 🔴 致命 | 40KB 栈溢出 |
| 5 | `main.c` UTF-16 LE 编码 | 🟠 严重 | 无法正常阅读，Git diff 异常 |
| 6 | PID 控制器重复实现两套 | 🟡 中等 | 维护困难，行为不一致 |
| 7 | 全局变量散落在多个文件中通过 `extern` 引用 | 🟡 中等 | 状态追踪困难 |
| 8 | 约 40% 代码被注释掉（死代码） | 🟡 中等 | 严重影响可读性 |
| 9 | `data_trans.c` 中 `=` 误写为 `==` | 🟠 严重 | 潜在逻辑 bug |

---

## 2. 架构总览

```
┌─────────────────────────────────────────────────────────┐
│                     main_v2.c                            │
│   HAL 初始化 → 时钟配置 → 外设初始化 → g_sys 初始化       │
│   → thread_manager_start_all() → 调度器运行               │
└──────────────────────┬──────────────────────────────────┘
                       │
     ┌─────────────────┼─────────────────┐
     ▼                 ▼                  ▼
┌─────────┐    ┌──────────────┐    ┌──────────────┐
│ sw_i2c  │    │ uart_manager │    │ system_state │
│ 总线抽象 │    │ UART 管理器   │    │  统一状态     │
│ +互斥锁  │    │ 独立缓冲区    │    │  单例 g_sys  │
└────┬────┘    └──────┬───────┘    └──────┬───────┘
     │                │                   │
┌────▼─────┐   ┌──────▼───────┐   ┌───────▼───────┐
│mpu6050_v2│   │  遥控器解析   │   │ pid_controller│
│ms5837_v2 │   │  超声波解析   │   │  统一 PID     │
│ 传感器驱动│   │  GPS 解析    │   │  微分先行     │
└──────────┘   └──────────────┘   └───────┬───────┘
                                          │
                                  ┌───────▼───────┐
                                  │motion_control │
                                  │   运动控制     │
                                  │ 手动/定深/航向 │
                                  └───────┬───────┘
                                          │
                                  ┌───────▼───────┐
                                  │ TIM2/TIM3 PWM │
                                  │  硬件输出      │
                                  └───────────────┘
```

**线程模型（优先级从高到低）：**

| 线程名 | 优先级 | 栈大小 | 频率 | 职责 |
|--------|:------:|:------:|:----:|------|
| motion | 10 | 2048 | 50Hz | 运动控制计算 + PWM 输出 |
| data_send | 12 | 1024 | 20Hz | 遥测数据发送 |
| imu | 15 | 1024 | 20Hz | MPU6050 姿态采集 |
| depth | 15 | 1024 | 5Hz | MS5837 深度采集 |
| ultrasound | 16 | 768 | 10Hz | 超声波测距 |
| ph | 16 | 768 | 10Hz | pH/水质 ADC 采集 |
| water | 16 | 512 | 2Hz | 水质传感器 |
| uart_rx | 18 | 512 | 事件驱动 | UART 中断接收管理 |
| led | 25 | 512 | 1Hz | 心跳指示灯 |

---

## 3. I2C 总线安全 — 核心改动

### 3.1 原始问题

原项目使用 **软件模拟 I2C**（GPIO 位操作），通过两个全局变量在运行时切换引脚：

```c
// IIC.c — 原始代码（有 Bug）
uint16_t IIC_SCL_PIN = GPIO_PIN_4;  // 全局变量，运行时被随意修改
uint16_t IIC_SDA_PIN = GPIO_PIN_5;

// 所有 I2C 操作宏都引用这两个全局变量
#define IIC_SCL_1()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_SET)
#define IIC_SDA_0()  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_RESET)
```

两个线程同时访问不同设备：

```c
// 线程 A: MPU6050 (PB4/PB5)
IIC_SCL_PIN = SCL_1_Pin;   // = PB4
IIC_SDA_PIN = SDA_1_Pin;   // = PB5
MPU_Init();                 // 但此时可能已被线程 B 改掉！

// 线程 B: MS5837 (PB0/PB1) — 同时运行
IIC_SCL_PIN = MS5837_SCL_Pin;  // = PB0  ← 覆盖了线程 A 的设置！
IIC_SDA_PIN = MS5837_SDA_Pin;  // = PB1
MS5837_30BA_ReSet();
```

**危险时序示例：**

```
T1: [线程A] IIC_SCL_PIN = PB4    // 设置为 MPU6050 引脚
T2: [线程B] IIC_SCL_PIN = PB0    // 覆盖！线程A的设置丢失
T3: [线程A] IIC_Start()          // 操作的是 PB0 而不是 PB4 → 数据错乱！
T4: [线程B] IIC_Send_Byte(...)   // 也在操作 PB0 → 总线冲突！
```

### 3.2 解决方案：`sw_i2c_bus` 总线抽象

**核心思想**：每个 I2C 设备拥有独立的总线实例，封装自己的引脚和互斥锁。

```
┌─────────────────────────────────┐    ┌─────────────────────────────────┐
│  sw_i2c_bus_t "i2c_imu"        │    │  sw_i2c_bus_t "i2c_depth"      │
│  ┌───────────────────────────┐ │    │  ┌───────────────────────────┐ │
│  │ gpio_port: GPIOB          │ │    │  │ gpio_port: GPIOB          │ │
│  │ scl_pin:   PB4            │ │    │  │ scl_pin:   PB0            │ │
│  │ sda_pin:   PB5            │ │    │  │ sda_pin:   PB1            │ │
│  │ mutex:     ✓              │ │    │  │ mutex:     ✓              │ │
│  └───────────────────────────┘ │    │  └───────────────────────────┘ │
│           │ 独立操作              │    │           │ 独立操作              │
│     ┌─────▼─────┐               │    │     ┌─────▼─────┐               │
│     │  MPU6050  │               │    │     │  MS5837   │               │
│     └───────────┘               │    │     └───────────┘               │
└─────────────────────────────────┘    └─────────────────────────────────┘
         两个总线完全独立，互不干扰，各自互斥锁只保护自己的操作
```

### 3.3 `sw_i2c_bus_t` 结构体

```c
typedef struct {
    GPIO_TypeDef *gpio_port;    // GPIO 端口 (GPIOA/GPIOB/...)
    uint16_t      scl_pin;      // SCL 引脚号
    uint16_t      sda_pin;      // SDA 引脚号
    uint32_t      delay_us;     // 半周期延时 (微秒)
    uint32_t      timeout_ms;   // ACK 超时
    rt_mutex_t    mutex;        // RT-Thread 互斥锁
    char          name[RT_NAME_MAX]; // 总线名称
} sw_i2c_bus_t;
```

### 3.4 关键 API

| 函数 | 说明 | 线程安全 |
|------|------|:--------:|
| `sw_i2c_init()` | 初始化总线（GPIO + 互斥锁） | ✓ |
| `sw_i2c_start()` | 发送起始信号 | 需手动加锁 |
| `sw_i2c_stop()` | 发送停止信号 | 需手动加锁 |
| `sw_i2c_send_byte()` | 发送一字节（含 ACK 检测） | 需手动加锁 |
| `sw_i2c_read_byte()` | 读取一字节（含 ACK/NACK） | 需手动加锁 |
| `sw_i2c_write_reg()` | **高级**：写寄存器（自动加锁） | ✓ |
| `sw_i2c_read_reg()` | **高级**：读寄存器（自动加锁） | ✓ |
| `sw_i2c_check_device()` | 检测设备是否存在 | ✓ |

### 3.5 使用示例

```c
// 初始化（在 system_state_init 中完成）
sw_i2c_init(&g_sys.i2c_bus_imu,  GPIOB, PB4, PB5, 400000, "i2c_imu");
sw_i2c_init(&g_sys.i2c_bus_depth, GPIOB, PB0, PB1, 400000, "i2c_dep");

// 简单操作（自动加锁）
sw_i2c_write_reg(&g_sys.i2c_bus_imu, MPU_ADDR, reg, &data, 1);

// 复合操作（手动控制锁的范围）
sw_i2c_lock(&g_sys.i2c_bus_depth);
sw_i2c_start(&g_sys.i2c_bus_depth);
sw_i2c_send_byte(&g_sys.i2c_bus_depth, ...);
sw_i2c_read_byte(&g_sys.i2c_bus_depth, ...);
sw_i2c_stop(&g_sys.i2c_bus_depth);
sw_i2c_unlock(&g_sys.i2c_bus_depth);
```

### 3.6 与原方案对比

| 特性 | 原方案 | 新方案 |
|------|--------|--------|
| 多设备支持 | 运行时切换全局引脚 | 各设备独立总线实例 |
| 线程安全 | ❌ 无保护 | ✓ RT-Thread 互斥锁 |
| 引脚冲突检测 | ❌ 无 | ✓ 编译期隔离 |
| 代码复用 | 宏定义绑定全局变量 | 函数参数化，可复用 |
| 新建设备 | 需新增全局变量 | 新 `sw_i2c_init()` 一行 |

---

## 4. UART 多路管理

### 4.1 原始问题

```c
// 原代码 — 致命 Bug
uint8_t data = 0;   // ← 唯一的接收字节，所有 UART 共用！

// 三路 UART 同时往同一个地址写
HAL_UART_Receive_IT(&huart1, &data, 1);  // 遥控器
HAL_UART_Receive_IT(&huart4, &data, 1);  // 超声波
HAL_UART_Receive_IT(&huart7, &data, 1);  // GPS

// 中断回调中
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    uint8_t received_byte = data;  // ← 不知道是哪个 UART 的数据！
    ...
}
```

**问题**：三路 UART 中断可能在任意时刻触发，都往 `data` 这个地址写，后到达的中断会覆盖先到达的数据。

### 4.2 解决方案：`uart_manager`

```c
// 每路 UART 拥有独立的上下文
typedef struct {
    UART_HandleTypeDef *huart;          // HAL 句柄
    volatile uint8_t    rx_byte;        // ← 独立的中断接收字节！
    uint8_t             rx_buffer[128]; //   独立的帧缓冲区
    uart_parser_cb_t    parser_cb;      //   独立的协议解析器
} uart_channel_ctx_t;

static uart_channel_ctx_t g_uart_ctx[4];  // 4 路独立通道
```

**初始化**：

```c
void uart_manager_init(void) {
    g_uart_ctx[UART_CH_USART1].huart = &huart1;  // 遥控器
    g_uart_ctx[UART_CH_UART4].huart  = &huart4;  // 超声波
    g_uart_ctx[UART_CH_UART5].huart  = &huart5;  // 深度
    g_uart_ctx[UART_CH_UART7].huart  = &huart7;  // GPS

    // 每路使用自己的 rx_byte
    for (int i = 0; i < 4; i++)
        HAL_UART_Receive_IT(g_uart_ctx[i].huart,
                           (uint8_t *)&g_uart_ctx[i].rx_byte, 1);
}
```

**中断回调**：

```c
void uart_rx_callback(UART_HandleTypeDef *huart) {
    // 根据 huart 找到对应通道
    uart_channel_ctx_t *ctx = find_channel(huart);
    
    uint8_t byte = ctx->rx_byte;  // ← 读取的是该通道独占的字节
    
    // 调用该通道的协议解析器
    ctx->parser_cb(ctx->rx_buffer, ctx->rx_count);
    
    // 重新启动中断接收（该通道自己的 rx_byte）
    HAL_UART_Receive_IT(huart, (uint8_t *)&ctx->rx_byte, 1);
}
```

---

## 5. 统一系统状态

### 5.1 原始状态管理

原项目中全局变量散落在至少 5 个文件中：

```
main.c:          encoder_count_1, encoder_flag, sem
thread_manager.c: pitch_1, roll_1, yaw_1, temp, jy901b, mpu6050, PS_2, ...
motion_control.c: pwm_1, pwm_2, pwm_3, pid_depth, ...
my_function.c:    extern 引用各种变量
data_trans.c:     extern 引用 PS_2, sum, head, len...
```

### 5.2 新方案：`system_state_t`

```c
typedef struct {
    // I2C 总线实例
    sw_i2c_bus_t i2c_bus_imu;       // MPU6050 专用
    sw_i2c_bus_t i2c_bus_depth;     // MS5837 专用

    // 传感器数据
    mpu6050_data_t     imu;         // MPU6050 姿态
    mpu6050_raw_t      imu_raw;     // MPU6050 原始值
    jy901b_data_t      jy901b;      // JY901B 姿态
    ms5837_data_t      depth_sensor;// MS5837 深度
    remote_data_t      remote;      // 遥控器
    ultrasound_data_t  ultrasound;  // 超声波
    water_quality_data_t water;     // pH/水质
    gps_data_t         gps;         // GPS

    // 控制输出
    float motor_left_speed;         // 左电机 PWM
    float motor_right_speed;        // 右电机 PWM
    float motor_back_speed;         // 后电机 PWM
    servo_angle_t      servo;       // 舵机角度

    // 系统模式
    system_mode_t      mode;        // 运行模式
    system_flags_t     flags;       // 运行标志

    // 深度
    true_depth_t       depth;       // 真实深度 + 偏移
} system_state_t;

// 全局唯一实例
extern system_state_t g_sys;
```

**好处**：
- 所有状态集中在一处，调试时只需看一个结构体
- 可以通过 `memcpy` 做状态快照用于日志
- 类型安全，IDE 自动补全 `g_sys.`
- 新增加传感器只需在结构体中添加字段

---

## 6. 统一 PID 控制器

### 6.1 原始问题

原项目中 PID 控制器重复实现了 **4 次**：

1. `my_function.h` 中 `MYPID` 结构体（声明但未使用）
2. `motion_control.c` 中 `PID` 结构体 + `pid_init()` + `pid_compute()`（简单版）
3. `motion_control.c` 中 `EnhancedPID_t` + `pid_init_enhanced()` + `pid_compute_enhanced()`
4. `motion_control_v2.h` 中 `pid_ctrl_t`（重构后唯一使用）

### 6.2 统一实现：`pid_ctrl_t`

```c
typedef struct {
    float kp, ki, kd;           // 增益
    float out_min, out_max;     // 输出限幅
    float integral_max;         // 积分饱和上限
    float setpoint;             // 设定值
    float integral;             // 积分累加
    float last_error;           // 上次测量值（微分先行用）
    float last_deriv;           // 微分低通滤波记忆
} pid_ctrl_t;
```

**特性**：

| 特性 | 说明 |
|------|------|
| **微分先行** (Derivative on Measurement) | 对测量值微分而非误差，避免设定值突变时的微分冲击 |
| **抗积分饱和** (Anti-Windup) | 积分累加受 `integral_max` 限制 |
| **输出限幅** | 最终输出钳位到 `[out_min, out_max]` |
| **微分低通滤波** | 抑制传感器高频噪声 |
| **在线调参** | `pid_tune()` 运行时调整增益 |

```c
// 使用示例
pid_ctrl_t depth_pid;
pid_init(&depth_pid, 1.0, 0.1, 0.01, 0.0, -200, 200);

float output = pid_compute(&depth_pid, measured_depth, dt);

// 在线调参
pid_tune(&depth_pid, 1.5, 0.05, 0.02);

// 模式切换时重置积分器
pid_reset(&depth_pid);
```

---

## 7. 运动控制重构

### 7.1 控制模式状态机

```
        ┌──────────┐
        │  MODE_   │  ← 默认模式
        │  MANUAL  │
        └────┬─────┘
             │ ch9 > 1.5
        ┌────▼─────┐
        │  MODE_   │
        │ DEPTH_   │  ← PID 定深
        │  HOLD    │
        └────┬─────┘
             │ ch9 < 1.5
        ┌────▼─────┐
        │  MODE_   │  ← 回到手动
        │  MANUAL  │
        └──────────┘

        任意模式 → MODE_FAILSAFE（遥控器断连/电机未解锁）
```

### 7.2 各模式控制策略

**手动模式 (`MODE_MANUAL`)**：
```
ch1 (前进/后退) → 推力映射到 M1/M2 推进器
ch2 (横移)      → 差速映射
ch3 (升降)      → 推力映射到 M3 推进器
ch4 (转向)      → 差速映射
```

**定深模式 (`MODE_DEPTH_HOLD`)**：
```
水平面: 手动遥控 (ch1/ch2/ch4)
深度:   PID 自动控制 (ch3 用于微调目标深度)
        measured = g_sys.depth_sensor.depth_m
        setpoint = g_sys.depth.target_depth
        output   → M3 推进器推力
```

---

## 8. 线程管理器重构

### 8.1 初始化流程

```
main()
  ├── HAL_Init()
  ├── SystemClock_Config()          // HSI + PLL → 480MHz
  ├── MX_GPIO_Init()                // GPIO 初始化
  ├── MX_ADC1_Init()                // ADC 初始化 (pH/水质)
  ├── MX_TIM2_Init()                // 舵机 PWM 定时器
  ├── MX_TIM3_Init()                // 电调 PWM 定时器
  ├── MX_TIM4_Init() / MX_TIM6_Init()
  ├── MX_USART1_UART_Init()         // 遥控器 UART
  ├── MX_USART2_UART_Init()         // 调试/遥测 UART
  ├── MX_UART4_Init()               // 超声波 UART
  ├── MX_UART5_Init()               // 深度传感器 UART
  ├── MX_UART7_Init()               // GPS UART
  ├── system_state_init()           // ← I2C 总线初始化在此
  │     ├── sw_i2c_init(&g_sys.i2c_bus_imu, ...)
  │     └── sw_i2c_init(&g_sys.i2c_bus_depth, ...)
  └── thread_manager_start_all()    // ← 创建并启动所有线程
        ├── rt_thread_create("led", ...)
        ├── rt_thread_create("imu", thread_imu, ...)
        ├── rt_thread_create("depth", thread_depth, ...)
        ├── rt_thread_create("uart_rx", thread_uart_rx, ...)
        ├── rt_thread_create("data_send", thread_data_send, ...)
        ├── rt_thread_create("ultrasnd", thread_ultrasound, ...)
        ├── rt_thread_create("ph", thread_ph, ...)
        ├── rt_thread_create("water", thread_water_quality, ...)
        └── rt_thread_create("motion", thread_motion_ctrl, ...)
```

---

## 9. 修复的 Bug 清单

| # | 文件 | 行号 | 类型 | 问题 | 修复 |
|---|------|:----:|------|------|------|
| 1 | `IIC.c` | 全局 | 🔴 致命 | 全局 `IIC_SCL_PIN/SDA_PIN` 无保护切换 | 重构为 `sw_i2c_bus` + 互斥锁 |
| 2 | `thread_manager.c` | ~145 | 🔴 致命 | 多 UART 共用 `data` 接收字节 | 重构为 `uart_manager` 独立缓冲区 |
| 3 | `board.c` | 9 | 🔴 致命 | `RT_HEAP_SIZE = 1024`（仅1KB） | 改为 16384（16KB） |
| 4 | `thread_manager.c` | ~320 | 🔴 致命 | `float jiaodu[10000]` 栈分配 40KB | 已移除 |
| 5 | `rtconfig.h` | 33 | 🟠 严重 | `RT_MAIN_THREAD_STACK_SIZE = 1024` | 改为 2048 |
| 6 | `data_trans.c` | 29 | 🟠 严重 | `(data=0x00)` 应为 `(data==0x00)` | 改为 `==` |
| 7 | `main.c` | 全部 | 🟠 严重 | UTF-16 LE 编码 | 新建 UTF-8 `main_v2.c` |
| 8 | `motion_control.c` | 多处 | 🟡 中等 | PID 重复实现 2 套 | 统一为 `pid_controller.c` |
| 9 | 多个文件 | 全局 | 🟡 中等 | 全局变量散落 extern 引用 | 统一为 `system_state_t g_sys` |

---

## 10. 文件变更对照表

### 10.1 新增文件

| 文件 | 行数 | 说明 |
|------|:----:|------|
| `my/my_head/sw_i2c.h` | 115 | 软件 I2C 总线抽象头文件 |
| `my/my_source/sw_i2c.c` | 260 | 软件 I2C 总线实现（GPIO 位操作 + 互斥锁） |
| `my/my_head/system_state.h` | 144 | 统一系统状态头文件 |
| `my/my_source/system_state.c` | 38 | 系统状态初始化 |
| `my/my_head/pid_controller.h` | 78 | 统一 PID 控制器头文件 |
| `my/my_source/pid_controller.c` | 76 | 统一 PID 控制器实现 |
| `my/my_head/uart_manager.h` | 80 | 多路 UART 管理头文件 |
| `my/my_source/uart_manager.c` | 190 | 多路 UART 管理实现 |
| `my/my_head/mpu6050_v2.h` | 74 | MPU6050 驱动重构版头文件 |
| `my/my_source/mpu6050_v2.c` | 155 | MPU6050 驱动重构版实现 |
| `my/my_head/ms5837_v2.h` | 95 | MS5837 驱动重构版头文件 |
| `my/my_source/ms5837_v2.c` | 210 | MS5837 驱动重构版实现 |
| `my/my_head/motion_control_v2.h` | 118 | 运动控制重构版头文件 |
| `my/my_source/motion_control_v2.c` | 260 | 运动控制重构版实现 |
| `my/my_head/thread_manager_v2.h` | 57 | 线程管理器重构版头文件 |
| `my/my_source/thread_manager_v2.c` | 380 | 线程管理器重构版实现 |
| `Core/Src/main_v2.c` | 160 | 新系统入口 (UTF-8) |
| `Core/Src/startup_stm32h750xx_gcc.S` | 300+ | GCC 启动文件 |
| `STM32H750VBTx_FLASH.ld` | 120 | GCC 链接脚本 |
| `Makefile` | 120 | GCC 构建脚本 |
| `REFACTOR_NOTES.md` | 120 | 重构说明 |
| `README.md` | 80 | 项目说明 |
| `.gitignore` | 50 | Git 忽略规则 |

### 10.2 修改的文件

| 文件 | 改动 | 说明 |
|------|------|------|
| `RT-Thread/rtconfig.h` | `RT_MAIN_THREAD_STACK_SIZE` 1024→2048 | 增大主线程栈 |
| `my/my_RTT/board.c` | `RT_HEAP_SIZE` 1024→16384 | 增大堆空间 |
| `my/my_source/data_trans.c` | `=` → `==` | 修复 bug |
| `my/my_source/motion_control.c` | `pid_init/pid_compute` 用 `#if 0` 禁用 | 避免链接冲突 |
| `my/my_source/thread_manager.c` | `HAL_UART_RxCpltCallback` 用 `#if 0` 禁用 | 避免链接冲突 |
| `my/my_source/app_uart.c` | `main()` 用 `#if 0` 禁用 | 避免链接冲突 |

### 10.3 保留的旧文件（向后兼容）

| 文件 | 保留原因 |
|------|----------|
| `my/my_source/my_function.c` | `Send_Data_Task()` 遥测组帧、`fputc()` printf 重定向 |
| `my/my_source/data_trans.c` | `SHIP_DT_LX_Data_Receive_Prepare()` 遥控器协议解析 |
| `my/my_source/my_GPS.c` | GPS NMEA 解析 |
| `my/my_source/my_adc.c` | ADC 辅助函数 |
| `my/my_source/my_it.c` | 编码器 EXTI 中断 |
| `my/my_source/inv_mpu.c` | DMP 库（姿态解算） |
| `my/my_source/inv_mpu_dmp_motion_driver.c` | DMP 运动驱动 |
| `my/my_source/IIC.c` | 旧 I2C 实现（保留但不再编译） |
| `my/my_source/mpu6050.c` | 旧 MPU6050 驱动（保留但不再编译） |
| `my/my_source/MS5837.c` | 旧 MS5837 驱动（保留但不再编译） |

---

## 11. API 参考

### 11.1 I2C 总线操作

```c
// 初始化
rt_err_t sw_i2c_init(sw_i2c_bus_t *bus, GPIO_TypeDef *port,
                     uint16_t scl, uint16_t sda,
                     uint32_t freq_hz, const char *name);

// 基本时序（需手动加锁）
void sw_i2c_start(sw_i2c_bus_t *bus);
void sw_i2c_stop(sw_i2c_bus_t *bus);
uint8_t sw_i2c_send_byte(sw_i2c_bus_t *bus, uint8_t data);
uint8_t sw_i2c_read_byte(sw_i2c_bus_t *bus, uint8_t ack);
uint8_t sw_i2c_wait_ack(sw_i2c_bus_t *bus);
void sw_i2c_send_ack(sw_i2c_bus_t *bus);
void sw_i2c_send_nack(sw_i2c_bus_t *bus);

// 高级操作（自动加锁）
uint8_t sw_i2c_write_reg(sw_i2c_bus_t *bus, uint8_t addr,
                         uint8_t reg, const uint8_t *data, uint8_t len);
uint8_t sw_i2c_read_reg(sw_i2c_bus_t *bus, uint8_t addr,
                        uint8_t reg, uint8_t *data, uint8_t len);
uint8_t sw_i2c_check_device(sw_i2c_bus_t *bus, uint8_t addr);

// 手动锁控制
rt_err_t sw_i2c_lock(sw_i2c_bus_t *bus);
rt_err_t sw_i2c_unlock(sw_i2c_bus_t *bus);
```

### 11.2 PID 控制器

```c
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float setpoint, float out_min, float out_max);
float pid_compute(pid_ctrl_t *pid, float measured, float dt);
void pid_reset(pid_ctrl_t *pid);
void pid_setpoint(pid_ctrl_t *pid, float sp);
void pid_tune(pid_ctrl_t *pid, float kp, float ki, float kd);
```

### 11.3 MPU6050 驱动

```c
uint8_t mpu6050_init(sw_i2c_bus_t *bus);
uint8_t mpu6050_set_gyro_fsr(sw_i2c_bus_t *bus, uint8_t fsr);
uint8_t mpu6050_set_accel_fsr(sw_i2c_bus_t *bus, uint8_t fsr);
uint8_t mpu6050_set_rate(sw_i2c_bus_t *bus, uint16_t rate);
uint8_t mpu6050_get_accel(sw_i2c_bus_t *bus, int16_t *ax, int16_t *ay, int16_t *az);
uint8_t mpu6050_get_gyro(sw_i2c_bus_t *bus, int16_t *gx, int16_t *gy, int16_t *gz);
int16_t mpu6050_get_temp(sw_i2c_bus_t *bus);
uint8_t mpu6050_get_id(sw_i2c_bus_t *bus);
```

### 11.4 MS5837 驱动

```c
typedef struct { sw_i2c_bus_t *bus; ms5837_cal_t cal;
    float fluid_density, depth_offset; uint8_t initialized; } ms5837_dev_t;

typedef struct { float temperature, pressure_mbar, depth_m; } ms5837_result_t;

uint8_t ms5837_init(ms5837_dev_t *dev, sw_i2c_bus_t *bus, float density);
void ms5837_reset(ms5837_dev_t *dev);
uint8_t ms5837_read_prom(ms5837_dev_t *dev);
uint8_t ms5837_crc4(const ms5837_dev_t *dev);
uint8_t ms5837_read(ms5837_dev_t *dev, ms5837_result_t *result);
void ms5837_set_offset(ms5837_dev_t *dev, float offset);
```

### 11.5 UART 管理器

```c
typedef enum { UART_CH_USART1, UART_CH_UART4,
               UART_CH_UART5, UART_CH_UART7 } uart_channel_t;

void uart_manager_init(void);
void uart_rx_callback(UART_HandleTypeDef *huart);
void uart_set_parser(uart_channel_t ch, uart_parser_cb_t cb);
```

### 11.6 运动控制

```c
void motion_ctrl_init(motion_ctrl_t *mc);
void motion_ctrl_update(motion_ctrl_t *mc, system_state_t *sys);
void motion_ctrl_set_mode(motion_ctrl_t *mc, system_mode_t mode);
void motion_ctrl_set_depth(motion_ctrl_t *mc, float depth_m);
void motion_ctrl_set_heading(motion_ctrl_t *mc, float heading_deg);
void motion_ctrl_apply_output(motion_ctrl_t *mc);
```

---

## 12. Keil 工程迁移步骤

### 步骤 1：添加新源文件

在 Keil Project 窗口中右键各组 → **Add Existing Files**：

| Keil 组 | 添加文件 |
|---------|----------|
| `my_source` | `sw_i2c.c`, `system_state.c`, `pid_controller.c`, `uart_manager.c` |
| `my_source` | `mpu6050_v2.c`, `ms5837_v2.c`, `motion_control_v2.c`, `thread_manager_v2.c` |
| `Core/Src` | `main_v2.c` |

### 步骤 2：排除旧文件编译

右键以下文件 → **Options for File** → ☑ **Exclude from Build**：

```
□ IIC.c
□ mpu6050.c
□ MS5837.c
□ motion_control.c
□ thread_manager.c
□ app_uart.c
□ Core/Src/main.c
```

### 步骤 3：添加头文件路径

**Project → Options → C/C++ → Include Paths** 添加：

```
..\my\my_head
```

### 步骤 4：同步修改的文件

确认以下文件已更新为最新版本：

- `RT-Thread/rtconfig.h`（主线程栈 2048）
- `my/my_RTT/board.c`（堆 16KB）
- `my/my_source/data_trans.c`（`=` → `==`）

### 步骤 5：编译验证

```
Project → Build Target (F7)
```

预期结果：**0 Error(s), 0 Warning(s)**。

---

> 📄 本文档也保存在项目仓库中：`REFACTOR_NOTES.md`  
> 🔗 GitHub: https://github.com/sanfenrujian/ROV-Control-System
