/**
 * @file    motion_control_v2.c
 * @brief   Motion controller implementation for the v2 runtime path.
 */

#include "motion_control_v2.h"
#include "tim.h"
#include <math.h>
#include <rtthread.h>
#include <string.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define DT_DEFAULT              0.02f
#define DT_MAX                  0.10f

#define REMOTE_MIN              0.0f
#define REMOTE_CENTER           1.0f
#define REMOTE_MAX              2.0f
#define REMOTE_DEADBAND         0.08f

#define HEADING_SLEW_RATE_DPS  60.0f
#define DEPTH_TRIM_RATE_MPS     0.35f
#define MAX_DEPTH_TARGET_M     100.0f

#define GYRO_Z_LSB_PER_DPS      16.4f

#define SURGE_MIX_GAIN          0.55f
#define SWAY_MIX_GAIN           0.25f
#define YAW_MIX_GAIN            0.45f
#define VERTICAL_THRUSTER_SPLIT 0.50f

#define SERVO_SURGE_GAIN_DEG   10.0f
#define SERVO_SWAY_GAIN_DEG    20.0f
#define SERVO_YAW_GAIN_DEG     15.0f
#define SERVO_HEAVE_GAIN_DEG   20.0f

static void manual_control(motion_ctrl_t *mc, system_state_t *sys);
static void depth_hold_control(motion_ctrl_t *mc, system_state_t *sys);
static void heading_hold_control(motion_ctrl_t *mc, system_state_t *sys);
static void failsafe_control(motion_ctrl_t *mc, system_state_t *sys);

static float wrap_angle_deg(float angle_deg);
static float channel_to_unit(float channel);
static float apply_deadband(float value, float deadband);
static uint8_t fetch_absolute_heading(system_state_t *sys, float *heading_deg);
static void update_heading_estimate(motion_ctrl_t *mc, system_state_t *sys);
static void apply_horizontal_mix(motion_ctrl_t *mc, float surge_cmd,
                                 float sway_cmd, float yaw_cmd);
static void apply_vertical_mix(motion_ctrl_t *mc, float heave_cmd);
static float compute_heading_yaw_cmd(motion_ctrl_t *mc, float yaw_input);
static void finalize_output(motion_ctrl_t *mc);

void motion_ctrl_init(motion_ctrl_t *mc)
{
    if (mc == NULL) return;

    memset(mc, 0, sizeof(motion_ctrl_t));

    pid_init(&mc->pid_depth, DEPTH_KP, DEPTH_KI, DEPTH_KD,
             0.0f, -200.0f, 200.0f);
    pid_init(&mc->pid_heading, HEADING_KP, HEADING_KI, HEADING_KD,
             0.0f, -100.0f, 100.0f);
    pid_init(&mc->pid_pitch, PITCH_KP, PITCH_KI, PITCH_KD,
             0.0f, -50.0f, 50.0f);
    pid_init(&mc->pid_roll, ROLL_KP, ROLL_KI, ROLL_KD,
             0.0f, -50.0f, 50.0f);

    mc->m1_pwm = MOTOR_NEUTRAL_PWM;
    mc->m2_pwm = MOTOR_NEUTRAL_PWM;
    mc->m3_pwm = MOTOR_NEUTRAL_PWM;
    mc->m4_pwm = MOTOR_NEUTRAL_PWM;

    mc->heading_estimate = 0.0f;
    mc->heading_has_absolute = 0;
    mc->last_tick = rt_tick_get();
}

void motion_ctrl_set_mode(motion_ctrl_t *mc, system_mode_t mode)
{
    if (mc == NULL) return;

    rt_kprintf("[CTRL] Mode: %d -> %d\r\n", g_sys.mode, mode);

    pid_reset(&mc->pid_depth);
    pid_reset(&mc->pid_heading);
    pid_reset(&mc->pid_pitch);
    pid_reset(&mc->pid_roll);

    g_sys.mode = mode;

    if (mode == MODE_DEPTH_HOLD)
    {
        mc->target_depth = g_sys.depth.true_depth;
        pid_setpoint(&mc->pid_depth, mc->target_depth);
        g_sys.depth.target_depth = mc->target_depth;
    }

    if (mode == MODE_HEADING_HOLD)
    {
        mc->target_heading = wrap_angle_deg(mc->heading_estimate);
        pid_setpoint(&mc->pid_heading, mc->target_heading);
    }
}

void motion_ctrl_set_depth(motion_ctrl_t *mc, float depth_m)
{
    if (mc == NULL) return;

    mc->target_depth = clampf(depth_m, 0.0f, MAX_DEPTH_TARGET_M);
    pid_setpoint(&mc->pid_depth, mc->target_depth);
    g_sys.depth.target_depth = mc->target_depth;
}

void motion_ctrl_set_heading(motion_ctrl_t *mc, float heading_deg)
{
    if (mc == NULL) return;

    mc->target_heading = wrap_angle_deg(heading_deg);
    pid_setpoint(&mc->pid_heading, mc->target_heading);
}

void motion_ctrl_update(motion_ctrl_t *mc, system_state_t *sys)
{
    if (mc == NULL || sys == NULL) return;

    {
        uint32_t now = rt_tick_get();
        mc->dt = (float)(now - mc->last_tick) / (float)RT_TICK_PER_SECOND;
        mc->last_tick = now;
    }

    if (mc->dt > DT_MAX) mc->dt = DT_DEFAULT;
    if (mc->dt <= 0.0f)  mc->dt = DT_DEFAULT;

    update_heading_estimate(mc, sys);

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

    sys->motor_left_speed      = mc->m1_pwm;
    sys->motor_right_speed     = mc->m2_pwm;
    sys->motor_back_speed      = mc->m3_pwm;
    sys->motor_back2_speed     = mc->m4_pwm;
    sys->servo.left_angle      = mc->m1_angle;
    sys->servo.right_angle     = mc->m2_angle;
    sys->servo.back_left_angle = mc->m3_angle;
    sys->servo.back_up_angle   = mc->m4_angle;
}

static void manual_control(motion_ctrl_t *mc, system_state_t *sys)
{
    remote_data_t *rc = &sys->remote;
    float surge = apply_deadband(channel_to_unit(rc->ch1), REMOTE_DEADBAND);
    float sway  = apply_deadband(channel_to_unit(rc->ch2), REMOTE_DEADBAND);
    float heave = apply_deadband(channel_to_unit(rc->ch3), REMOTE_DEADBAND);
    float yaw   = apply_deadband(channel_to_unit(rc->ch4), REMOTE_DEADBAND);

    apply_horizontal_mix(mc, surge, sway, yaw);
    apply_vertical_mix(mc, heave);
    finalize_output(mc);
}

static void depth_hold_control(motion_ctrl_t *mc, system_state_t *sys)
{
    remote_data_t *rc = &sys->remote;
    float surge = apply_deadband(channel_to_unit(rc->ch1), REMOTE_DEADBAND);
    float sway  = apply_deadband(channel_to_unit(rc->ch2), REMOTE_DEADBAND);
    float heave_trim = apply_deadband(channel_to_unit(rc->ch3), REMOTE_DEADBAND);
    float yaw_input  = apply_deadband(channel_to_unit(rc->ch4), REMOTE_DEADBAND);
    float yaw_cmd    = yaw_input;
    float depth_output;

    if (fabsf(heave_trim) > 0.0f)
    {
        motion_ctrl_set_depth(mc,
            mc->target_depth - heave_trim * DEPTH_TRIM_RATE_MPS * mc->dt);
    }

    if (rc->ch10 > 1.5f)
    {
        yaw_cmd = compute_heading_yaw_cmd(mc, yaw_input);
    }

    apply_horizontal_mix(mc, surge, sway, yaw_cmd);

    depth_output = pid_compute(&mc->pid_depth, sys->depth.true_depth, mc->dt);
    apply_vertical_mix(mc, clampf(depth_output / 500.0f, -1.0f, 1.0f));
    finalize_output(mc);
}

static void heading_hold_control(motion_ctrl_t *mc, system_state_t *sys)
{
    remote_data_t *rc = &sys->remote;
    float surge = apply_deadband(channel_to_unit(rc->ch1), REMOTE_DEADBAND);
    float sway  = apply_deadband(channel_to_unit(rc->ch2), REMOTE_DEADBAND);
    float heave = apply_deadband(channel_to_unit(rc->ch3), REMOTE_DEADBAND);
    float yaw_input = apply_deadband(channel_to_unit(rc->ch4), REMOTE_DEADBAND);
    float yaw_cmd = compute_heading_yaw_cmd(mc, yaw_input);

    apply_horizontal_mix(mc, surge, sway, yaw_cmd);
    apply_vertical_mix(mc, heave);
    finalize_output(mc);
}

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

void motion_ctrl_apply_output(motion_ctrl_t *mc)
{
    uint16_t pwm1;
    uint16_t pwm2;
    uint16_t pwm3;
    uint16_t pwm4;

    if (mc == NULL) return;

    pwm1 = (uint16_t)(mc->m1_angle * 2000.0f / 360.0f + 1500.0f);
    pwm2 = (uint16_t)(mc->m2_angle * 2000.0f / 360.0f + 1500.0f);
    pwm3 = (uint16_t)(mc->m3_angle * 2000.0f / 360.0f + 1500.0f);
    pwm4 = (uint16_t)(mc->m4_angle * 2000.0f / 360.0f + 1500.0f);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm4);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)mc->m1_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)mc->m2_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)mc->m3_pwm);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t)mc->m4_pwm);
}

static float wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static float channel_to_unit(float channel)
{
    return clampf(channel, REMOTE_MIN, REMOTE_MAX) - REMOTE_CENTER;
}

static float apply_deadband(float value, float deadband)
{
    if (fabsf(value) <= deadband)
        return 0.0f;

    if (value > 0.0f)
        return (value - deadband) / (1.0f - deadband);

    return (value + deadband) / (1.0f - deadband);
}

static uint8_t fetch_absolute_heading(system_state_t *sys, float *heading_deg)
{
    if (sys == NULL || heading_deg == NULL)
        return 0;

    if (fabsf(sys->jy901b.pitch) > 0.001f ||
        fabsf(sys->jy901b.roll)  > 0.001f ||
        fabsf(sys->jy901b.yaw)   > 0.001f)
    {
        *heading_deg = wrap_angle_deg(sys->jy901b.yaw);
        return 1;
    }

    if (fabsf(sys->imu.pitch) > 0.001f ||
        fabsf(sys->imu.roll)  > 0.001f ||
        fabsf(sys->imu.yaw)   > 0.001f)
    {
        *heading_deg = wrap_angle_deg(sys->imu.yaw);
        return 1;
    }

    return 0;
}

static void update_heading_estimate(motion_ctrl_t *mc, system_state_t *sys)
{
    float absolute_heading = 0.0f;

    if (fetch_absolute_heading(sys, &absolute_heading))
    {
        mc->heading_estimate = absolute_heading;
        mc->heading_has_absolute = 1;
        return;
    }

    mc->heading_has_absolute = 0;
    mc->heading_estimate = wrap_angle_deg(
        mc->heading_estimate +
        ((float)sys->imu_raw.gyro_z / GYRO_Z_LSB_PER_DPS) * mc->dt);
}

static void apply_horizontal_mix(motion_ctrl_t *mc, float surge_cmd,
                                 float sway_cmd, float yaw_cmd)
{
    float m1_mix = surge_cmd * SURGE_MIX_GAIN
                 + sway_cmd  * SWAY_MIX_GAIN
                 + yaw_cmd   * YAW_MIX_GAIN;
    float m2_mix = surge_cmd * SURGE_MIX_GAIN
                 - sway_cmd  * SWAY_MIX_GAIN
                 - yaw_cmd   * YAW_MIX_GAIN;

    mc->m1_pwm = MOTOR_NEUTRAL_PWM + m1_mix * 500.0f;
    mc->m2_pwm = MOTOR_NEUTRAL_PWM + m2_mix * 500.0f;

    mc->m1_angle = surge_cmd * SERVO_SURGE_GAIN_DEG
                 + sway_cmd  * SERVO_SWAY_GAIN_DEG
                 + yaw_cmd   * SERVO_YAW_GAIN_DEG;
    mc->m2_angle = surge_cmd * SERVO_SURGE_GAIN_DEG
                 - sway_cmd  * SERVO_SWAY_GAIN_DEG
                 - yaw_cmd   * SERVO_YAW_GAIN_DEG;
}

static void apply_vertical_mix(motion_ctrl_t *mc, float heave_cmd)
{
    float thrust_delta = heave_cmd * 500.0f * VERTICAL_THRUSTER_SPLIT;
    float servo_angle = heave_cmd * SERVO_HEAVE_GAIN_DEG;

    mc->m3_pwm = MOTOR_NEUTRAL_PWM + thrust_delta;
    mc->m4_pwm = MOTOR_NEUTRAL_PWM + thrust_delta;
    mc->m3_angle = servo_angle;
    mc->m4_angle = servo_angle;
}

static float compute_heading_yaw_cmd(motion_ctrl_t *mc, float yaw_input)
{
    float wrapped_error;
    float wrapped_measured;
    float yaw_output;

    if (fabsf(yaw_input) > 0.0f)
    {
        motion_ctrl_set_heading(
            mc,
            mc->target_heading + yaw_input * HEADING_SLEW_RATE_DPS * mc->dt);
    }

    wrapped_error = wrap_angle_deg(mc->target_heading - mc->heading_estimate);
    wrapped_measured = mc->target_heading - wrapped_error;
    yaw_output = pid_compute(&mc->pid_heading, wrapped_measured, mc->dt);

    return clampf(yaw_output / 100.0f, -1.0f, 1.0f);
}

static void finalize_output(motion_ctrl_t *mc)
{
    mc->m1_pwm = clampf(mc->m1_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m2_pwm = clampf(mc->m2_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m3_pwm = clampf(mc->m3_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    mc->m4_pwm = clampf(mc->m4_pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);

    mc->m1_angle = clampf(mc->m1_angle, -SERVO_ANGLE_MAX, SERVO_ANGLE_MAX);
    mc->m2_angle = clampf(mc->m2_angle, -SERVO_ANGLE_MAX, SERVO_ANGLE_MAX);
    mc->m3_angle = clampf(mc->m3_angle, -SERVO_ANGLE_MAX, SERVO_ANGLE_MAX);
    mc->m4_angle = clampf(mc->m4_angle, -SERVO_ANGLE_MAX, SERVO_ANGLE_MAX);
}
