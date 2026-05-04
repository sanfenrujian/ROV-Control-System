/**
 * @file    motion_control_v2.c
 * @brief   水下机器人运动控制实现
 * 
 * 控制策略：
 *   - 手动模式：直接映射遥控器通道到推进器推力和舵机角度
 *   - 定深模式：PID 控制深度，同时允许水平面遥控
 *   - 航向锁定：PID 控制航向，同时允许其他遥控
 * 
 * 推进器布局（矢量推进）：
 *   M1 (左前):  前进 + 转向
 *   M2 (右前):  前进 - 转向
 *   M3 (后):    升降 + 俯仰
 *   M4 (后上):  辅助升降
 */

#include "motion_control_v2.h"
#include "tim.h"
#include <math.h>
#include <rtthread.h>
#include <string.h>

/* 外部 TIM 句柄 */
extern TIM_HandleTypeDef htim2;  /* 舵机 PWM */
extern TIM_HandleTypeDef htim3;  /* 电调 PWM */

/* ========================== 内部常量 ========================== */

#define DT_DEFAULT    0.02f   /* 默认控制周期 20ms (50Hz) */
#define DT_MAX        0.10f   /* 最大允许周期 */

/* ========================== 初始化 ========================== */

void motion_ctrl_init(motion_ctrl_t *mc)
{
    if (mc == NULL) return;

    memset(mc, 0, sizeof(motion_ctrl_t));

    /* 初始化深度 PID */
    pid_init(&mc->pid_depth, DEPTH_KP, DEPTH_KI, DEPTH_KD,
             0.0f, -200.0f, 200.0f);

    /* 初始化航向 PID */
    pid_init(&mc->pid_heading, HEADING_KP, HEADING_KI, HEADING_KD,
             0.0f, -100.0f, 100.0f);

    /* 初始化俯仰 PID */
    pid_init(&mc->pid_pitch, PITCH_KP, PITCH_KI, PITCH_KD,
             0.0f, -50.0f, 50.0f);

    /* 初始化横滚 PID */
    pid_init(&mc->pid_roll, ROLL_KP, ROLL_KI, ROLL_KD,
             0.0f, -50.0f, 50.0f);

    /* 默认中位 */
    mc->m1_pwm = MOTOR_NEUTRAL_PWM;
    mc->m2_pwm = MOTOR_NEUTRAL_PWM;
    mc->m3_pwm = MOTOR_NEUTRAL_PWM;
    mc->m4_pwm = MOTOR_NEUTRAL_PWM;

    mc->last_tick = rt_tick_get();
}

/* ========================== 模式切换 ========================== */

void motion_ctrl_set_mode(motion_ctrl_t *mc, system_mode_t mode)
{
    if (mc == NULL) return;

    rt_kprintf("[CTRL] Mode: %d -> %d\r\n", g_sys.mode, mode);

    /* 模式切换时重置积分器 */
    pid_reset(&mc->pid_depth);
    pid_reset(&mc->pid_heading);
    pid_reset(&mc->pid_pitch);
    pid_reset(&mc->pid_roll);

    g_sys.mode = mode;

    /* 切换到定深模式时，保持当前深度为目标 */
    if (mode == MODE_DEPTH_HOLD)
    {
        mc->target_depth = g_sys.depth.true_depth;
        pid_setpoint(&mc->pid_depth, mc->target_depth);
    }

    /* 切换到航向锁定时，保持当前航向 */
    if (mode == MODE_HEADING_HOLD)
    {
        mc->target_heading = g_sys.imu.yaw;
        pid_setpoint(&mc->pid_heading, mc->target_heading);
    }
}

void motion_ctrl_set_depth(motion_ctrl_t *mc, float depth_m)
{
    if (mc == NULL) return;
    mc->target_depth = depth_m;
    pid_setpoint(&mc->pid_depth, depth_m);
}

void motion_ctrl_set_heading(motion_ctrl_t *mc, float heading_deg)
{
    if (mc == NULL) return;
    mc->target_heading = heading_deg;
    pid_setpoint(&mc->pid_heading, heading_deg);
}

/* ========================== 各模式前向声明 ========================== */
static void manual_control(motion_ctrl_t *mc, system_state_t *sys);
static void depth_hold_control(motion_ctrl_t *mc, system_state_t *sys);
static void heading_hold_control(motion_ctrl_t *mc, system_state_t *sys);
static void failsafe_control(motion_ctrl_t *mc, system_state_t *sys);

/* ========================== 控制更新 ========================== */

void motion_ctrl_update(motion_ctrl_t *mc, system_state_t *sys)
{
    if (mc == NULL || sys == NULL) return;

    /* 计算控制周期 */
    uint32_t now = rt_tick_get();
    mc->dt = (float)(now - mc->last_tick) / (float)RT_TICK_PER_SECOND;
    mc->last_tick = now;

    if (mc->dt > DT_MAX) mc->dt = DT_DEFAULT;
    if (mc->dt <= 0.0f)  mc->dt = DT_DEFAULT;

    /* 根据模式执行控制 */
    switch (sys->mode)
    {
    case MODE_MANUAL:
        manual_control(mc, sys);
        break;

    case MODE_DEPTH_HOLD:
        depth_hold_control(mc, sys);
        break;

    case MODE_HEADING_HOLD:
        heading_hold_control(mc, sys);
        break;

    case MODE_FAILSAFE:
        failsafe_control(mc, sys);
        break;

    default:
        manual_control(mc, sys);
        break;
    }

    /* 保存到全局状态 */
    sys->motor_left_speed   = mc->m1_pwm;
    sys->motor_right_speed  = mc->m2_pwm;
    sys->motor_back_speed   = mc->m3_pwm;
    sys->motor_back2_speed  = mc->m4_pwm;
    sys->servo.left_angle   = mc->m1_angle;
    sys->servo.right_angle  = mc->m2_angle;
    sys->servo.back_left_angle  = mc->m3_angle;
    sys->servo.back_up_angle    = mc->m4_angle;
}

/* ========================== 各模式实现 ========================== */

/**
 * 手动遥控模式：直接映射遥控器通道
 * 
 * 遥控器映射：
 *   ch1: 前进/后退 (1.0=中位, >1=前进, <1=后退)
 *   ch2: 横移
 *   ch3: 升降
 *   ch4: 转向
 *   ch9: 定深使能
 *   ch10: 待机使能
 */
static void manual_control(motion_ctrl_t *mc, system_state_t *sys)
{
    remote_data_t *rc = &sys->remote;

    /* 基础推力映射 (遥控器值 0~2 映射到 PWM 1000~2000) */
    float thrust_x = (rc->ch1 - 1.0f) * 500.0f;  /* 前进推力 */
    float thrust_y = (rc->ch2 - 1.0f) * 500.0f;  /* 横移推力 */
    float thrust_z = (rc->ch3 - 1.0f) * 500.0f;  /* 升降推力 */

    /* 前向推力分配到左右推进器 */
    mc->m1_pwm = MOTOR_NEUTRAL_PWM + thrust_x * 0.5f + thrust_y * 0.25f;
    mc->m2_pwm = MOTOR_NEUTRAL_PWM + thrust_x * 0.5f - thrust_y * 0.25f;

    /* 升降分配到后推进器 */
    mc->m3_pwm = MOTOR_NEUTRAL_PWM + thrust_z;

    /* 舵机角度（用于矢量推进 - 根据实际机械结构调整） */
    mc->m1_angle = clampf(thrust_x * 0.02f + thrust_y * 0.04f, -90.0f, 90.0f);
    mc->m2_angle = clampf(thrust_x * 0.02f - thrust_y * 0.04f, -90.0f, 90.0f);
    mc->m3_angle = clampf(thrust_z * 0.04f, -90.0f, 90.0f);
    mc->m4_angle = 0.0f;

    /* 全局限幅 */
    mc->m1_pwm = clampf(mc->m1_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m2_pwm = clampf(mc->m2_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m3_pwm = clampf(mc->m3_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m4_pwm = MOTOR_NEUTRAL_PWM;
}

/**
 * 定深控制模式
 * 水平面手动遥控 + 深度 PID 自动控制
 */
static void depth_hold_control(motion_ctrl_t *mc, system_state_t *sys)
{
    remote_data_t *rc = &sys->remote;

    /* 水平面：手动遥控 */
    float thrust_x = (rc->ch1 - 1.0f) * 500.0f;
    float thrust_y = (rc->ch2 - 1.0f) * 500.0f;

    mc->m1_pwm = MOTOR_NEUTRAL_PWM + thrust_x * 0.5f + thrust_y * 0.25f;
    mc->m2_pwm = MOTOR_NEUTRAL_PWM + thrust_x * 0.5f - thrust_y * 0.25f;

    /* 深度：PID 控制 */
    float depth_output = pid_compute(&mc->pid_depth,
                                     sys->depth.true_depth, mc->dt);

    mc->m3_pwm = MOTOR_NEUTRAL_PWM + depth_output;
    mc->m4_pwm = MOTOR_NEUTRAL_PWM;

    mc->m1_angle = clampf(thrust_x * 0.02f + thrust_y * 0.04f, -90.0f, 90.0f);
    mc->m2_angle = clampf(thrust_x * 0.02f - thrust_y * 0.04f, -90.0f, 90.0f);
    mc->m3_angle = 0.0f;
    mc->m4_angle = 0.0f;

    /* 全局限幅 */
    mc->m1_pwm = clampf(mc->m1_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m2_pwm = clampf(mc->m2_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m3_pwm = clampf(mc->m3_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m4_pwm = clampf(mc->m4_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
}

/**
 * 航向锁定模式（占位）
 */
static void heading_hold_control(motion_ctrl_t *mc, system_state_t *sys)
{
    /* TODO: 实现基于 JY901B yaw 的航向 PID 控制 */
    manual_control(mc, sys);
}

/**
 * 失效保护：所有电机停止
 */
static void failsafe_control(motion_ctrl_t *mc, system_state_t *sys)
{
    (void)sys;
    mc->m1_pwm = MOTOR_NEUTRAL_PWM;
    mc->m2_pwm = MOTOR_NEUTRAL_PWM;
    mc->m3_pwm = MOTOR_NEUTRAL_PWM;
    mc->m4_pwm = MOTOR_NEUTRAL_PWM;
    mc->m1_angle = 0.0f;
    mc->m2_angle = 0.0f;
    mc->m3_angle = 0.0f;
    mc->m4_angle = 0.0f;
}

/* ========================== 硬件输出 ========================== */

void motion_ctrl_apply_output(motion_ctrl_t *mc)
{
    if (mc == NULL) return;

    /* 舵机 PWM (TIM2, CH1-CH4)
     * 角度 0° ~ 180° 映射到 PWM 500 ~ 2500
     * 角度 = (PWM - 1500) * 360 / 2000
     * PWM  = 角度 * 2000 / 360 + 1500
     */
    uint16_t pwm1 = (uint16_t)(mc->m1_angle * 2000.0f / 360.0f + 1500.0f);
    uint16_t pwm2 = (uint16_t)(mc->m2_angle * 2000.0f / 360.0f + 1500.0f);
    uint16_t pwm3 = (uint16_t)(mc->m3_angle * 2000.0f / 360.0f + 1500.0f);
    uint16_t pwm4 = (uint16_t)(mc->m4_angle * 2000.0f / 360.0f + 1500.0f);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm4);

    /* 电调 PWM (TIM3, CH1-CH4)
     * 1000us = 停止/反向最大
     * 1500us = 中位/停止
     * 2000us = 正向最大
     */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)mc->m1_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)mc->m2_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)mc->m3_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)mc->m4_pwm);
}
