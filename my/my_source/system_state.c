/**
 * @file    system_state.c
 * @brief   全局系统状态初始化
 */

#include "system_state.h"
#include "main.h"       /* 获取引脚定义 */
#include <string.h>

/** 全局系统状态单例 */
system_state_t g_sys;

void system_state_init(void)
{
    /* 清零整个结构体 */
    memset(&g_sys, 0, sizeof(system_state_t));

    /* --- 初始化 I2C 总线 --- */
    /* MPU6050: PB4(SCL), PB5(SDA), 400kHz */
    sw_i2c_init(&g_sys.i2c_bus_imu,
                GPIOB,
                SCL_1_Pin,   /* PB4 - 定义在 main.h */
                SDA_1_Pin,   /* PB5 - 定义在 main.h */
                400000,
                "i2c_imu");

    /* MS5837: PB0(SCL), PB1(SDA), 400kHz */
    sw_i2c_init(&g_sys.i2c_bus_depth,
                GPIOB,
                MS5837_SCL_Pin,  /* PB0 */
                MS5837_SDA_Pin,  /* PB1 */
                400000,
                "i2c_dep");

    /* --- 默认模式 --- */
    g_sys.mode = MODE_MANUAL;
    g_sys.flags.initialized = 1;

    /* --- 传感器默认值 --- */
    g_sys.depth.offset = 0.0f;
}
