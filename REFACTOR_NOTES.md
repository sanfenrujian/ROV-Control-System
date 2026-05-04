/**
 * ============================ 重构说明文档 ============================
 * 
 * 项目：水下机器人（ROV）运动控制系统
 * MCU：STM32H750VBT6
 * RTOS：RT-Thread 3.1.5
 * 
 * ============================ I2C 共享问题解决方案 ============================
 * 
 * 【原始问题】
 *   原项目使用软件 I2C（位操作），通过全局变量 IIC_SCL_PIN 和 IIC_SDA_PIN
 *   在不同线程中动态切换引脚来访问不同设备：
 *     - MPU6050:  PB4(SCL) / PB5(SDA)
 *     - MS5837:   PB0(SCL) / PB1(SDA)
 * 
 *   问题1：两个线程同时操作 I2C 引脚，无任何保护
 *          -> 可能导致数据错乱、总线冲突
 * 
 *   问题2：全局变量 IIC_SCL_PIN/SDA_PIN 在运行时被修改
 *          -> 线程A刚设置好PB4/PB5，线程B就切成了PB0/PB1
 * 
 *   问题3：IIC.c 中的宏 (IIC_SCL_1, IIC_SDA_0 等) 直接引用全局变量
 *          -> 所有操作共享同一个 GPIO 端口和引脚变量
 * 
 * 【解决方案：sw_i2c_bus 总线抽象】
 * 
 *   每个 I2C 设备拥有独立的 sw_i2c_bus_t 实例：
 * 
 *     sw_i2c_bus_t bus_imu;    // MPU6050 专属
 *     sw_i2c_bus_t bus_depth;  // MS5837 专属
 * 
 *   每个实例封装了：
 *     - GPIO 端口和引脚 (gpio_port, scl_pin, sda_pin)
 *     - RT-Thread 互斥锁 (rt_mutex_t)
 *     - 所有 I2C 操作原语 (start/stop/send_byte/read_byte)
 * 
 *   线程安全保证：
 *     线程A: sw_i2c_write_reg(&bus_imu, ...)
 *            -> sw_i2c_lock() 获取 bus_imu 的互斥锁
 *            -> 执行 I2C 操作（操作 PB4/PB5）
 *            -> sw_i2c_unlock() 释放锁
 * 
 *     线程B: sw_i2c_write_reg(&bus_depth, ...)
 *            -> sw_i2c_lock() 获取 bus_depth 的互斥锁
 *            -> 执行 I2C 操作（操作 PB0/PB1）
 *            -> sw_i2c_unlock() 释放锁
 * 
 *   两个线程操作不同的总线实例，互不干扰，无需等待！
 *   只有同一总线上的操作才会互斥。
 * 
 * 
 * ============================ 新增文件清单 ============================
 * 
 * my/my_head/sw_i2c.h              - 软件I2C总线抽象头文件
 * my/my_source/sw_i2c.c            - 软件I2C总线实现
 * my/my_head/system_state.h        - 统一系统状态头文件
 * my/my_source/system_state.c      - 系统状态初始化
 * my/my_head/pid_controller.h      - 统一PID控制器头文件
 * my/my_source/pid_controller.c    - 统一PID控制器实现
 * my/my_head/uart_manager.h        - 多路UART管理头文件
 * my/my_source/uart_manager.c      - 多路UART管理实现
 * my/my_head/mpu6050_v2.h          - MPU6050驱动(重构版)
 * my/my_source/mpu6050_v2.c        - MPU6050驱动实现(重构版)
 * my/my_head/ms5837_v2.h           - MS5837驱动(重构版)
 * my/my_source/ms5837_v2.c         - MS5837驱动实现(重构版)
 * my/my_head/motion_control_v2.h   - 运动控制头文件(重构版)
 * my/my_source/motion_control_v2.c - 运动控制实现(重构版)
 * my/my_head/thread_manager_v2.h   - 线程管理器头文件(重构版)
 * my/my_source/thread_manager_v2.c - 线程管理器实现(重构版)
 * Core/Src/main_v2.c               - 系统入口(重构版)
 * 
 * ============================ 修改文件清单 ============================
 * 
 * RT-Thread/rtconfig.h             - 增大主线程栈 (2048)
 * my/my_RTT/board.c                - 增大堆空间 (16KB)
 * 
 * ============================ 迁移步骤 ============================
 * 
 * 1. 在 Keil/IAR 工程中添加所有 *_v2.c 源文件
 * 2. 添加 my/my_head/ 到 include paths
 * 3. 将 Core/Src/main.c 替换为 Core/Src/main_v2.c
 *    或将 main_v2.c 的内容复制到 main.c（需转换编码为 UTF-8）
 * 4. 确认 HAL_UART_RxCpltCallback 已转发到 uart_rx_callback()
 * 5. 确认 rtconfig.h 中 RT_USING_MUTEX 已启用 (已启用)
 * 6. 重新编译
 * 
 * ============================ 原文件保留策略 ============================
 * 
 * 旧文件（IIC.c, mpu6050.c, MS5837.c 等）建议保留到 Git 历史中，
 * 但在工程中排除编译。如果 DMP 库（inv_mpu_dmp_motion_driver）仍需要，
 * 需将其 I2C 读写函数适配为 sw_i2c_bus 接口。
 * 
 * ============================ 修复的致命问题 ============================
 * 
 * 1. [已修复] 多UART共用 data 变量 -> 每路独立 rx_byte
 * 2. [已修复] I2C 引脚无保护共享 -> sw_i2c_bus + 互斥锁
 * 3. [已修复] 堆空间仅1KB -> 增大到16KB
 * 4. [已修复] main 线程栈仅1024 -> 增大到2048
 * 5. [已修复] jiaodu[10000] 40KB栈溢出 -> 移除
 * 6. [已修复] main.c UTF-16编码 -> 新建 UTF-8 main_v2.c
 */
