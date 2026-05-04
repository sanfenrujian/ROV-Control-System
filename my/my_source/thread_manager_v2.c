/**
 * @file    thread_manager_v2.c
 * @brief   线程管理器实现（重构版）
 * 
 * 包含所有线程入口函数和创建逻辑。
 * 
 * 与旧版的区别：
 *   - 不再使用全局变量，改用 g_sys 统一状态
 *   - MPU6050 和 MS5837 各自使用独立的 sw_i2c_bus
 *   - UART 通过 uart_manager 统一管理
 *   - 运动控制使用统一的 pid_ctrl_t
 *   - 无死代码
 */

#include "thread_manager_v2.h"
#include "system_state.h"
#include "uart_manager.h"
#include "mpu6050_v2.h"
#include "ms5837_v2.h"
#include "motion_control_v2.h"
#include "pid_controller.h"
#include "sw_i2c.h"
#include "ota.h"                     /* OTA 在线升级 */

#include "main.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "my_function.h"           /* 旧类型定义 (ps2DATATYPE, JY901B_DataType 等) */
#include "mpu6050.h"               /* MPU_Read_Len / MPU_Write_Len 桩函数需要 */

#include <stdio.h>
#include <string.h>

/* ========================== OTA 相关 ========================== */
static uint32_t ota_trigger_tick = 0;   /* OTA 触发计时 */
#define OTA_TRIGGER_DELAY   3000        /* 长按 3 秒进入 OTA (tick数) */
#define OTA_BUFFER_SIZE     64          /* OTA 帧接收缓冲 */

/* ========================== 全局运动控制器实例 ========================== */
static motion_ctrl_t g_mc;
static ms5837_dev_t  g_ms5837_dev;

/* ========================== 向后兼容：旧代码所需的全局变量 ========================== */
/*
 * 以下变量被旧模块 (data_trans.c, my_function.c, my_it.c) 引用，
 * 作为 extern 引用。此处提供定义以避免 L6218E 链接错误。
 * 这些变量应在新架构稳定后逐步移除。
 */

/* data_trans.c 所需 */
ps2DATATYPE     PS_2;            /* 遥控器数据 (旧类型) */
uint8_t         head, tail;      /* 协议解析帧头/尾 */
uint8_t         id, len;         /* 协议帧 ID 和长度 */
uint8_t         sum;             /* 协议帧校验和 */

/* my_function.c 所需 */
JY901B_DataType  jy901b;         /* JY901B 姿态 (旧类型) */
JY901B_Temp      jy901b_t;       /* JY901B 温度 (旧类型) */
MS5837_DATATYPE  ms5837;         /* MS5837 原始数据 (旧类型) */
servo_angle_Datatype  send_angle;/* 舵机角度 (旧类型) */
trueshendu_DATATYPE  true_depth; /* 真实深度 (旧类型) */

/* my_it.c 所需 */
uint32_t encoder_count_1;
uint8_t  encoder_flag;

/* ========================== DMP 库 I2C 桩函数 ========================== */
/*
 * inv_mpu.c / inv_mpu_dmp_motion_driver.c 依赖旧 I2C API。
 * 此处提供桩函数桥接到新的 sw_i2c_bus，或直接返回错误以禁用 DMP。
 * DMP 功能当前未使用（系统用 JY901B 获取姿态），故返回 1 (失败)。
 */

void IIC_GPIO_Init(void)
{
    /* 旧 I2C GPIO 初始化 - 已由 sw_i2c_init 替代，此处为空 */
}

uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    /* 桥接到 sw_i2c_bus */
    return sw_i2c_read_reg(&g_sys.i2c_bus_imu, addr, reg, buf, len);
}

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    /* 桥接到 sw_i2c_bus */
    return sw_i2c_write_reg(&g_sys.i2c_bus_imu, addr, reg, buf, len);
}

/*
 * printf 重定向已在 my_function.c 中定义，此处不再重复，
 * 避免 Keil 链接时 L6200E multiply defined 错误。
 */

/* ========================== 线程1: LED 心跳 ========================== */
static void thread_led(void *param)
{
    (void)param;

    while (1)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        rt_thread_mdelay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        rt_thread_mdelay(500);
    }
}

/* ========================== 线程2: MPU6050 姿态采集 ========================== */
static void thread_imu(void *param)
{
    (void)param;

    sw_i2c_bus_t *bus = &g_sys.i2c_bus_imu;

    /* 初始化 MPU6050 */
    rt_kprintf("[IMU] Initializing MPU6050...\r\n");

    while (mpu6050_init(bus) != 0)
    {
        rt_kprintf("[IMU] MPU6050 init failed, retrying...\r\n");
        rt_thread_mdelay(500);
    }

    rt_kprintf("[IMU] MPU6050 OK\r\n");

    /* 主循环：20Hz 采集 */
    while (1)
    {
        /* 读取加速度计 */
        mpu6050_get_accel(bus, &g_sys.imu_raw.accel_x,
                          &g_sys.imu_raw.accel_y,
                          &g_sys.imu_raw.accel_z);

        /* 读取陀螺仪 */
        mpu6050_get_gyro(bus, &g_sys.imu_raw.gyro_x,
                         &g_sys.imu_raw.gyro_y,
                         &g_sys.imu_raw.gyro_z);

        /* 读取温度 */
        g_sys.imu.temp = mpu6050_get_temp(bus) / 100.0f;

        /* TODO: 如果需要 DMP 姿态解算，在此处调用
         * mpu6050_dmp_get_data(bus, &g_sys.imu.pitch,
         *                      &g_sys.imu.roll, &g_sys.imu.yaw);
         * 当前从 JY901B 获取姿态数据
         */

        rt_thread_mdelay(50);
    }
}

/* ========================== 线程3: MS5837 深度采集 ========================== */
static void thread_depth(void *param)
{
    (void)param;

    sw_i2c_bus_t *bus = &g_sys.i2c_bus_depth;

    rt_kprintf("[DEPTH] Initializing MS5837...\r\n");

    /* 初始化 MS5837 */
    while (ms5837_init(&g_ms5837_dev, bus, 1.0f) != 0)
    {
        rt_kprintf("[DEPTH] MS5837 init failed, retrying...\r\n");
        rt_thread_mdelay(500);
    }

    /* 水面校准：将当前压力作为零位 */
    ms5837_result_t result;
    if (ms5837_read(&g_ms5837_dev, &result) == 0)
    {
        g_sys.depth.offset = result.depth_m;
        rt_kprintf("[DEPTH] Zeroed at %.2f m\r\n", result.depth_m);
    }

    rt_kprintf("[DEPTH] MS5837 OK\r\n");

    /* 主循环：5Hz 采集 */
    while (1)
    {
        if (ms5837_read(&g_ms5837_dev, &result) == 0)
        {
            g_sys.depth_sensor.depth_m       = result.depth_m;
            g_sys.depth_sensor.pressure_mbar = result.pressure_mbar;
            g_sys.depth_sensor.temperature   = result.temperature;

            g_sys.depth.true_depth = result.depth_m;

            /* 校准目标深度（首次有效读数） */
            if (g_sys.depth.target_depth < 0.01f)
            {
                g_sys.depth.target_depth = result.depth_m;
            }
        }

        rt_thread_mdelay(200);
    }
}

/* ========================== 线程4: UART 接收管理 ========================== */
static void thread_uart_rx(void *param)
{
    (void)param;

    /* 初始化 UART 管理器（自动启动各通道中断接收） */
    uart_manager_init();

    rt_kprintf("[UART] All channels initialized\r\n");

    /* 此线程仅负责初始化，之后休眠 */
    while (1)
    {
        rt_thread_mdelay(1000);
    }
}

/* ========================== 线程5: 数据发送 ========================== */
static void thread_data_send(void *param)
{
    (void)param;

    /*
     * 遥测数据格式（简化版）：
     * 帧头 0xAA | ID | LEN | 数据... | CHECKSUM
     * 
     * 包含：姿态角、深度、温度、电机输出、模式等
     */

    while (1)
    {
        /* 构造遥测帧并通过 USART2 发送 */
        uint8_t buf[64];
        uint8_t idx = 0;

        buf[idx++] = 0xAA;  /* 帧头 */
        buf[idx++] = 0x02;  /* 遥测 ID */
        buf[idx++] = 0;     /* 长度占位 */

        /* 姿态角 (3个float = 12字节) */
        memcpy(&buf[idx], &g_sys.imu.pitch, 4); idx += 4;
        memcpy(&buf[idx], &g_sys.imu.roll,  4); idx += 4;
        memcpy(&buf[idx], &g_sys.imu.yaw,   4); idx += 4;

        /* 深度 (1个float = 4字节) */
        memcpy(&buf[idx], &g_sys.depth_sensor.depth_m, 4); idx += 4;

        /* 电机输出 (4个float = 16字节) */
        memcpy(&buf[idx], &g_sys.motor_left_speed,  4); idx += 4;
        memcpy(&buf[idx], &g_sys.motor_right_speed, 4); idx += 4;
        memcpy(&buf[idx], &g_sys.motor_back_speed,  4); idx += 4;
        memcpy(&buf[idx], &g_sys.motor_back2_speed, 4); idx += 4;

        /* 模式 (1字节) */
        buf[idx++] = (uint8_t)g_sys.mode;

        /* 回填长度 */
        buf[2] = idx - 3;

        /* 计算校验和 */
        uint8_t sum = 0;
        for (uint8_t i = 0; i < idx; i++)
            sum += buf[i];
        buf[idx++] = sum;

        /* 发送 */
        HAL_UART_Transmit(&huart2, buf, idx, 100);

        rt_thread_mdelay(50);  /* 20Hz 发送 */
    }
}

/* ========================== 线程6: 超声波采集 ========================== */
static void thread_ultrasound(void *param)
{
    (void)param;

    /* 超声波通过 UART4 接收，数据由 uart_manager 自动解析到 g_sys */

    while (1)
    {
        /* 周期性发送触发脉冲（根据传感器型号调整） */
        rt_thread_mdelay(100);  /* 10Hz */
    }
}

/* ========================== 线程7: pH/水质采集 ========================== */
static void thread_ph(void *param)
{
    (void)param;

    extern ADC_HandleTypeDef hadc1;

    /* pH 传感器通过 ADC1 通道7/8 采集 */
    while (1)
    {
        float adc_vals[2];

        for (int i = 0; i < 2; i++)
        {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 10);

            if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
            {
                adc_vals[i] = HAL_ADC_GetValue(&hadc1) * 3.3f / 4096.0f;
            }

            HAL_ADC_Stop(&hadc1);
        }

        /* pH 计算 */
        g_sys.water.ph_value = 6.5f - adc_vals[1];

        /* TDS 计算 */
        float v = adc_vals[0];
        g_sys.water.tds = v * v * v * 66.71f - 127.93f * v * v + 428.7f * v;

        rt_thread_mdelay(100);  /* 10Hz */
    }
}

/* ========================== 线程8: 水质传感器 ========================== */
static void thread_water_quality(void *param)
{
    (void)param;

    /* 占位：水质传感器数据采集 */
    while (1)
    {
        rt_thread_mdelay(500);
    }
}

/* ========================== 线程9: 运动控制 ========================== */
static void thread_motion_ctrl(void *param)
{
    (void)param;

    /* 初始化运动控制器 */
    motion_ctrl_init(&g_mc);

    rt_kprintf("[CTRL] Motion controller ready\r\n");

    /* 电机解锁逻辑 */
    uint8_t motors_locked = 1;

    /* 初始化 OTA 模块 */
    ota_init(115200);

    while (1)
    {
        /* 检查遥控器解锁信号（ch8 > 0.8 表示解锁） */
        if (motors_locked && g_sys.remote.ch8 > 0.8f)
        {
            motors_locked = 0;
            g_sys.flags.motors_armed = 1;
            rt_kprintf("[CTRL] Motors ARMED\r\n");
        }

        /* ====== OTA 触发检测 ======
         * 安全策略：ch7 拉低 (油门最低) 持续 3 秒进入 OTA 模式
         * 防止水中误触发导致失联
         */
        if (g_sys.remote.ch7 < 0.2f)
        {
            if (ota_trigger_tick == 0)
                ota_trigger_tick = rt_tick_get();
            else if (rt_tick_get() - ota_trigger_tick >
                     rt_tick_from_millisecond(OTA_TRIGGER_DELAY))
            {
                rt_kprintf("[OTA] Triggered! Entering OTA mode...\r\n");
                motors_locked = 1;
                g_sys.flags.motors_armed = 0;

                /* 停止电机输出 */
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);

                /* 进入 OTA 接收模式 */
                ota_start();

                rt_kprintf("[OTA] Send firmware via USART2 now...\r\n");
                rt_kprintf("[OTA] Protocol: AA 55 CMD LEN_H LEN_L [DATA...] CHK\r\n");

                /* OTA 接收循环 */
                uint8_t ota_buf[OTA_BUFFER_SIZE];
                while (ota_get_state() < OTA_STATE_COMPLETE &&
                       ota_get_state() != OTA_STATE_ERROR)
                {
                    /* 从 USART2 读取一帧 */
                    uint16_t rx_len = 0;
                    for (int i = 0; i < OTA_BUFFER_SIZE; i++)
                    {
                        if (HAL_UART_Receive(&huart2, &ota_buf[i], 1, 10) == HAL_OK)
                        {
                            rx_len = i + 1;
                            /* 检测帧尾 (最后一个字节是校验和, 帧头已知) */
                            if (rx_len >= 5 && ota_buf[0] == OTA_SYNC_BYTE1)
                            {
                                uint16_t dlen = ota_buf[3] | ((uint16_t)ota_buf[4] << 8);
                                if (rx_len >= 5 + dlen + 1)
                                    break;
                            }
                        }
                        else
                        {
                            break; /* 超时 */
                        }
                    }

                    if (rx_len > 0)
                    {
                        uint8_t ack = ota_process_frame(ota_buf, rx_len);
                        /* 回复应答 */
                        uint8_t resp[] = { OTA_SYNC_BYTE1, OTA_SYNC_BYTE2, ack, 0 };
                        HAL_UART_Transmit(&huart2, resp, 4, 10);
                    }

                    rt_thread_mdelay(10);
                }

                /* OTA 完成或出错，看门狗或手动复位 */
                if (ota_get_state() == OTA_STATE_ERROR)
                {
                    rt_kprintf("[OTA] FAILED! Reset to retry.\r\n");
                    HAL_Delay(3000);
                }

                ota_trigger_tick = 0;
            }
        }
        else
        {
            ota_trigger_tick = 0;  /* 复位计时 */
        }

        /* 电机未解锁时使用失效保护 */
        if (!g_sys.flags.motors_armed)
        {
            g_sys.mode = MODE_FAILSAFE;
        }
        /* 检查遥控器模式切换 */
        else
        {
            if (g_sys.remote.ch9 > 1.5f)  /* ch9 > 1.5: 定深模式 */
            {
                if (g_sys.mode != MODE_DEPTH_HOLD)
                    motion_ctrl_set_mode(&g_mc, MODE_DEPTH_HOLD);
            }
            else  /* 手动模式 */
            {
                if (g_sys.mode != MODE_MANUAL)
                    motion_ctrl_set_mode(&g_mc, MODE_MANUAL);
            }
        }

        /* 执行控制计算 */
        motion_ctrl_update(&g_mc, &g_sys);

        /* 写入硬件 */
        motion_ctrl_apply_output(&g_mc);

        rt_thread_mdelay(20);  /* 50Hz 控制频率 */
    }
}

/* ========================== 线程创建 ========================== */

void thread_manager_start_all(void)
{
    rt_thread_t tid;

    /* LED 心跳 */
    tid = rt_thread_create("led",
                           thread_led, RT_NULL,
                           STACK_LED, PRIO_LED, 10);
    if (tid) rt_thread_startup(tid);

    /* MPU6050 姿态采集 */
    tid = rt_thread_create("imu",
                           thread_imu, RT_NULL,
                           STACK_IMU, PRIO_IMU, 10);
    if (tid) rt_thread_startup(tid);

    /* MS5837 深度采集 */
    tid = rt_thread_create("depth",
                           thread_depth, RT_NULL,
                           STACK_DEPTH_SENSOR, PRIO_DEPTH_SENSOR, 10);
    if (tid) rt_thread_startup(tid);

    /* UART 接收管理 */
    tid = rt_thread_create("uart_rx",
                           thread_uart_rx, RT_NULL,
                           STACK_UART_RX, PRIO_UART_RX, 10);
    if (tid) rt_thread_startup(tid);

    /* 数据发送 */
    tid = rt_thread_create("data_send",
                           thread_data_send, RT_NULL,
                           STACK_DATA_SEND, PRIO_DATA_SEND, 10);
    if (tid) rt_thread_startup(tid);

    /* 超声波 */
    tid = rt_thread_create("ultrasnd",
                           thread_ultrasound, RT_NULL,
                           STACK_ULTRASOUND, PRIO_ULTRASOUND, 10);
    if (tid) rt_thread_startup(tid);

    /* pH 采集 */
    tid = rt_thread_create("ph",
                           thread_ph, RT_NULL,
                           STACK_PH, PRIO_PH, 10);
    if (tid) rt_thread_startup(tid);

    /* 水质 */
    tid = rt_thread_create("water",
                           thread_water_quality, RT_NULL,
                           STACK_WATER_QUALITY, PRIO_WATER_QUALITY, 10);
    if (tid) rt_thread_startup(tid);

    /* 运动控制（最后启动，确保传感器线程已就绪） */
    tid = rt_thread_create("motion",
                           thread_motion_ctrl, RT_NULL,
                           STACK_MOTION_CTRL, PRIO_MOTION_CTRL, 10);
    if (tid) rt_thread_startup(tid);

    rt_kprintf("[SYS] All threads started\r\n");
}
