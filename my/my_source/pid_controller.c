/**
 * @file    pid_controller.c
 * @brief   统一 PID 控制器实现
 * 
 * 特性：
 *   - 微分先行（Derivative on Measurement）：对测量值而非误差微分，避免设定值突变时的微分冲击
 *   - 抗积分饱和（Anti-Windup）：积分累加受 integral_max 限制
 *   - 输出限幅：最终输出钳位到 [out_min, out_max]
 *   - 微分低通滤波：抑制高频噪声
 */

#include "pid_controller.h"
#include <string.h>

/* 微分低通滤波系数 (0~1, 越小滤波越强) */
#define PID_DERIV_LPF_ALPHA  0.3f

void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float setpoint, float out_min, float out_max)
{
    if (pid == NULL) return;

    memset(pid, 0, sizeof(pid_ctrl_t));

    pid->kp       = kp;
    pid->ki       = ki;
    pid->kd       = kd;
    pid->setpoint = setpoint;
    pid->out_min  = out_min;
    pid->out_max  = out_max;

    /* 默认积分限幅为输出范围的50% */
    pid->integral_max = (out_max - out_min) * 0.5f;

    pid->initialized = 1;
}

float pid_compute(pid_ctrl_t *pid, float measured, float dt)
{
    if (pid == NULL || !pid->initialized || dt <= 0.0f)
        return 0.0f;

    /* 1. 计算误差 */
    float error = pid->setpoint - measured;

    /* 2. 比例项 */
    float p_term = pid->kp * error;

    /* 3. 积分项（带抗饱和） */
    pid->integral += error * dt;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->ki * pid->integral;

    /* 4. 微分项（微分先行 + 低通滤波） */
    /* 对测量值的变化求微分，而不是对误差求微分 */
    float derivative = (measured - pid->last_error) / dt;
    pid->last_deriv = PID_DERIV_LPF_ALPHA * derivative
                    + (1.0f - PID_DERIV_LPF_ALPHA) * pid->last_deriv;
    float d_term = -pid->kd * pid->last_deriv;  /* 负号：测量值增大时应减小输出 */

    /* 5. 合成输出 */
    float output = p_term + i_term + d_term;

    /* 6. 输出限幅 */
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    /* 7. 保存误差用于下次微分 */
    pid->last_error = measured;

    return output;
}
