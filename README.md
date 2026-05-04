# ROV 水下机器人运动控制系统

基于 STM32H750VBT6 + RT-Thread 3.1.5 的水下机器人（ROV）嵌入式控制固件。

## 硬件平台

- **MCU**: STM32H750VBT6 (Cortex-M7, 480MHz)
- **RTOS**: RT-Thread 3.1.5
- **传感器**: MPU6050 (IMU), MS5837 (深度/水压), JY901B (姿态), GPS, 超声波, pH
- **执行器**: 4路推进器 (TIM2 舵机 + TIM3 电调 PWM)
- **通信**: 7路 UART

## 项目结构

```
├── Core/                    # HAL 外设配置 + 系统入口
│   ├── Inc/                 # 头文件 (main.h, gpio.h, tim.h ...)
│   └── Src/                 # 源文件 (main_v2.c, gpio.c, ...)
├── Drivers/                 # STM32H7 HAL 驱动库
│   ├── CMSIS/
│   └── STM32H7xx_HAL_Driver/
├── my/
│   ├── my_head/             # 应用层头文件
│   │   ├── sw_i2c.h         #  软件 I2C 总线抽象 (带互斥锁)
│   │   ├── system_state.h   #  统一系统状态
│   │   ├── pid_controller.h #  统一 PID 控制器
│   │   ├── uart_manager.h   #  多路 UART 管理
│   │   ├── mpu6050_v2.h     #  MPU6050 驱动 (重构版)
│   │   ├── ms5837_v2.h      #  MS5837 驱动 (重构版)
│   │   ├── motion_control_v2.h # 运动控制 (重构版)
│   │   └── thread_manager_v2.h # 线程管理 (重构版)
│   └── my_source/           # 应用层源文件
├── RT-Thread/               # RT-Thread 配置文件
├── RTT 3.1.5/               # RT-Thread 内核源码
├── MDK-ARM/                 # Keil MDK 工程
└── STM32H750VBTx_FLASH.ld   # GCC 链接脚本
```

## 控制模式

| 模式 | 说明 |
|------|------|
| MODE_MANUAL | 手动遥控 |
| MODE_DEPTH_HOLD | 定深保持 (PID 控制) |
| MODE_HEADING_HOLD | 航向锁定 |
| MODE_FAILSAFE | 失效保护 |

## 编译

### Keil MDK
打开 `MDK-ARM/ship.uvprojx`，编译目标 `ship`。

### GCC (VS Code)
```bash
# 安装 ARM GNU Toolchain
# https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads

make -j8
```

## 重构说明

v2 版本相比原始代码的主要改进：

1. **I2C 总线安全**: 使用 `sw_i2c_bus` 抽象，每个设备独立总线实例 + RT-Thread 互斥锁
2. **UART 安全**: 每路 UART 独立接收缓冲区，消除多路共用 bug
3. **状态管理**: `system_state_t` 统一全局状态，替代散落的 extern 变量
4. **PID 统一**: `pid_ctrl_t` 替代重复的 PID 实现
5. **堆栈扩大**: 堆从 1KB 增至 16KB，消除栈溢出风险

详细重构文档见 `REFACTOR_NOTES.md`。
