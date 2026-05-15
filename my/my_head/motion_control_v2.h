/**
 * @file    motion_control_v2.h
 * @brief   水下机器人运动控制模块（重构版）
 * 
 * 控制模式：
 *   MODE_MANUAL     - 手动遥控（推进器推力映射）
 *   MODE_DEPTH_HOLD - 定深控制（PID 控制升降）
 *   MODE_HEADING_HOLD - 航向锁定
 * 
 * 与原 motion_control.h 的区别：
 *   - 统一使用 pid_ctrl_t 替代重复的 PID 实现
 *   - 函数参数明确化，减少全局变量依赖
 *   - 控制模式状态机清晰
 */

#ifndef __MOTION_CONTROL_V2_H__
#define __MOTION_CONTROL_V2_H__

#include "system_state.h"
#include "pid_controller.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 控制参数 ========================== */

/** 推进器布局常量 */
#define MOTOR_NEUTRAL_PWM   1500.0f  /* 电机中位 PWM */
#define MOTOR_MIN_PWM        600.0f  /* 电机最小 PWM */
#define MOTOR_MAX_PWM       2400.0f  /* 电机最大 PWM */
#define SERVO_ANGLE_MAX       90.0f  /* 舵机最大角度（度） */

/** 默认 PID 参数 */
#define DEPTH_KP   1.0f
#define DEPTH_KI   0.1f
#define DEPTH_KD   0.01f

#define HEADING_KP 0.5f
#define HEADING_KI 0.05f
#define HEADING_KD 0.01f

#define PITCH_KP   1.0f
#define PITCH_KI   0.01f
#define PITCH_KD   0.0f

#define ROLL_KP    0.2f
#define ROLL_KI    0.01f
#define ROLL_KD    0.0f

/* ========================== 运动控制上下文 ========================== */

typedef struct {
    /* --- PID 控制器 --- */
    pid_ctrl_t pid_depth;     /* 定深 PID */
    pid_ctrl_t pid_heading;   /* 航向 PID */
    pid_ctrl_t pid_pitch;     /* 俯仰 PID */
    pid_ctrl_t pid_roll;      /* 横滚 PID */

    /* --- 目标值 --- */
    float target_depth;       /* 目标深度 (m) */
    float target_heading;     /* 目标航向 (度) */
    float heading_estimate;   /* 当前航向估计 (度) */

    /* --- 电机输出 --- */
    float m1_pwm;             /* 左推进器 PWM */
    float m2_pwm;             /* 右推进器 PWM */
    float m3_pwm;             /* 后推进器 PWM */
    float m4_pwm;             /* 后上推进器 PWM */
    float m1_angle;           /* 左舵机角度 */
    float m2_angle;           /* 右舵机角度 */
    float m3_angle;           /* 后左舵机角度 */
    float m4_angle;           /* 后上舵机角度 */

    /* --- 控制时间 --- */
    uint32_t last_tick;       /* 上次控制周期 tick */
    float    dt;              /* 控制周期 (秒) */
    uint8_t  heading_has_absolute; /* 当前是否有绝对航向输入 */
} motion_ctrl_t;

/* ========================== API ========================== */

/**
 * @brief 初始化运动控制器
 */
void motion_ctrl_init(motion_ctrl_t *mc);

/**
 * @brief 主控制循环（在 motion_control_thread 中每周期调用一次）
 * @param mc   控制器实例
 * @param sys  系统状态（包含传感器数据和模式）
 */
void motion_ctrl_update(motion_ctrl_t *mc, system_state_t *sys);

/**
 * @brief 切换控制模式
 */
void motion_ctrl_set_mode(motion_ctrl_t *mc, system_mode_t mode);

/**
 * @brief 设置目标深度（仅定深模式有效）
 */
void motion_ctrl_set_depth(motion_ctrl_t *mc, float depth_m);

/**
 * @brief 设置目标航向（仅航向锁定模式有效）
 */
void motion_ctrl_set_heading(motion_ctrl_t *mc, float heading_deg);

/**
 * @brief 将电机输出写入硬件 PWM
 */
void motion_ctrl_apply_output(motion_ctrl_t *mc);

/* ---------- 辅助函数 ---------- */

/** 限幅函数 */
static inline float clampf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/** 线性映射 */
static inline float mapf(float val, float in_min, float in_max,
                         float out_min, float out_max)
{
    if (in_max == in_min) return out_min;
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_CONTROL_V2_H__ */
