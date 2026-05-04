/**
 * @file    pid_controller.h
 * @brief   统一PID控制器（带抗积分饱和、输出限幅、微分先行）
 */

#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** PID 控制器实例 */
typedef struct {
    /* --- 增益参数 --- */
    float kp;           /* 比例系数 */
    float ki;           /* 积分系数 */
    float kd;           /* 微分系数 */

    /* --- 限幅 --- */
    float out_min;      /* 输出下限 */
    float out_max;      /* 输出上限 */
    float integral_max; /* 积分饱和上限（绝对值） */

    /* --- 状态 --- */
    float setpoint;     /* 设定值 */
    float integral;     /* 积分累加 */
    float last_error;   /* 上次误差（微分项用） */
    float last_deriv;   /* 微分低通滤波记忆 */

    /* --- 标志 --- */
    uint8_t initialized : 1;
} pid_ctrl_t;

/* ========================== API ========================== */

/**
 * @brief 初始化 PID 控制器
 * @param pid      控制器指针
 * @param kp,ki,kd 增益参数
 * @param setpoint 初始设定值
 * @param out_min  输出下限
 * @param out_max  输出上限
 */
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float setpoint, float out_min, float out_max);

/**
 * @brief PID 计算（微分先行 + 抗积分饱和 + 输出限幅）
 * @param pid           控制器指针
 * @param measured      当前测量值
 * @param dt            采样周期（秒）
 * @return              控制输出（已限幅到 [out_min, out_max]）
 */
float pid_compute(pid_ctrl_t *pid, float measured, float dt);

/**
 * @brief 设置新的设定值
 */
static inline void pid_setpoint(pid_ctrl_t *pid, float sp)
{
    pid->setpoint = sp;
}

/**
 * @brief 重置积分器（用于模式切换）
 */
static inline void pid_reset(pid_ctrl_t *pid)
{
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
    pid->last_deriv = 0.0f;
}

/**
 * @brief 在线调整增益
 */
static inline void pid_tune(pid_ctrl_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H__ */
