/**
 * @file    system_state.h
 * @brief   统一系统状态管理
 * 
 * 将原先分散在多个文件中、通过 extern 引用的全局变量
 * 集中管理到一个结构体中，便于维护、调试和状态快照。
 */

#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__

#include "sw_i2c.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 传感器数据结构 ========================== */

/** MPU6050 姿态数据 (DMP解算结果) */
typedef struct {
    float pitch;    /* 俯仰角 (度) */
    float roll;     /* 横滚角 (度) */
    float yaw;      /* 偏航角 (度) */
    float temp;     /* 温度 (°C) */
} mpu6050_data_t;

/** MPU6050 原始数据 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_t;

/** JY901B 姿态传感器数据 */
typedef struct {
    float pitch;    /* 俯仰角 (度) */
    float roll;     /* 横滚角 (度) */
    float yaw;      /* 偏航角 (度) */
    float temp;     /* 温度 (°C) */
} jy901b_data_t;

/** MS5837 深度/水压传感器数据 */
typedef struct {
    float depth_m;      /* 深度 (米) */
    float pressure_mbar;/* 压力 (毫巴) */
    float temperature;  /* 温度 (°C) */
} ms5837_data_t;

/** 遥控器/手柄数据 (PS2协议或自定义) */
typedef struct {
    float ch1;  /* 通道1: 前进/后退 */
    float ch2;  /* 通道2: 横移 */
    float ch3;  /* 通道3: 升降 */
    float ch4;  /* 通道4: 转向 */
    float ch5;  /* 通道5: 备用 */
    float ch6;  /* 通道6: 备用 */
    float ch7;  /* 通道7: OTA 触发 */
    float ch8;  /* 通道8: 电机解锁 */
    float ch9;  /* 通道9: 定深保持使能 */
    float ch10; /* 通道10: 航向锁定使能 */
} remote_data_t;

/** 超声波传感器数据 */
typedef struct {
    float distance_m;   /* 距离 (米) */
    uint8_t valid;      /* 数据有效标志 */
} ultrasound_data_t;

/** pH + 水质传感器数据 */
typedef struct {
    float ph_value;     /* pH值 */
    float tds;          /* TDS值 (水质) */
} water_quality_data_t;

/** GPS 数据 */
typedef struct {
    char   utc_time[16];    /* UTC时间 */
    char   latitude[16];     /* 纬度 */
    char   latitude_ns[2];   /* N/S */
    char   longitude[16];    /* 经度 */
    char   longitude_ew[2];  /* E/W */
    uint8_t is_valid;        /* 定位有效 */
    uint8_t is_updated;      /* 数据已更新 */
} gps_data_t;

/** 深度传感器原始字节数据 (兼容旧协议) */
typedef struct {
    uint8_t bytes[4];
    float   depth;
} depth_raw_t;

/** 舵机角度数据 */
typedef struct {
    float left_angle;       /* 左推进器角度 */
    float right_angle;      /* 右推进器角度 */
    float back_left_angle;  /* 后左推进器角度 */
    float back_up_angle;    /* 后上推进器角度 */
} servo_angle_t;

/** 真实深度 (经过零位校准) */
typedef struct {
    float true_depth;   /* 真实深度 (米) */
    float target_depth; /* 目标深度 (米) */
    float offset;       /* 零位偏移 */
} true_depth_t;

/* ========================== 系统模式与状态 ========================== */

/** 系统运行模式 */
typedef enum {
    MODE_MANUAL       = 0,  /* 手动遥控模式 */
    MODE_DEPTH_HOLD   = 1,  /* 定深模式 */
    MODE_HEADING_HOLD = 2,  /* 航向锁定模式 */
    MODE_AUTONOMOUS   = 3,  /* 自主航行模式 */
    MODE_FAILSAFE     = 4,  /* 失效保护模式 */
    MODE_COUNT
} system_mode_t;

/** 系统运行标志 */
typedef struct {
    uint8_t initialized   : 1;  /* 系统已初始化 */
    uint8_t motors_armed  : 1;  /* 电机已解锁 */
    uint8_t failsafe      : 1;  /* 失效保护激活 */
    uint8_t data_logging  : 1;  /* 数据记录中 */
    uint8_t reserved      : 4;
} system_flags_t;

/** 统一系统状态 */
typedef struct {
    /* --- I2C 总线实例 --- */
    sw_i2c_bus_t i2c_bus_imu;       /* MPU6050 专用 I2C 总线 (PB4/PB5) */
    sw_i2c_bus_t i2c_bus_depth;     /* MS5837 专用 I2C 总线 (PB0/PB1) */

    /* --- 传感器数据 --- */
    mpu6050_data_t     imu;         /* MPU6050 姿态数据 */
    mpu6050_raw_t      imu_raw;     /* MPU6050 原始数据 */
    jy901b_data_t      jy901b;      /* JY901B 姿态数据 */
    ms5837_data_t      depth_sensor;/* MS5837 深度数据 */
    remote_data_t      remote;      /* 遥控器数据 */
    ultrasound_data_t  ultrasound;  /* 超声波数据 */
    water_quality_data_t water;     /* pH/水质数据 */
    gps_data_t         gps;         /* GPS数据 */
    depth_raw_t        depth_raw;   /* 深度原始字节 */

    /* --- 控制数据 --- */
    servo_angle_t      servo;       /* 舵机角度 */
    true_depth_t       depth;       /* 真实深度 */

    /* --- 电机输出 --- */
    float motor_left_speed;         /* 左电机 PWM (1000-2000) */
    float motor_right_speed;        /* 右电机 PWM */
    float motor_back_speed;         /* 后电机 PWM */
    float motor_back2_speed;        /* 后上电机 PWM */

    /* --- 系统模式 --- */
    system_mode_t      mode;        /* 当前运行模式 */
    system_flags_t     flags;       /* 运行标志 */

    /* --- 编码器 --- */
    uint32_t encoder_count_1;
    uint32_t encoder_count_2;

} system_state_t;

/* ========================== 全局声明 ========================== */

/** 全局系统状态单例 */
extern system_state_t g_sys;

/**
 * @brief 初始化系统状态（包括 I2C 总线、传感器默认值等）
 */
void system_state_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_STATE_H__ */
