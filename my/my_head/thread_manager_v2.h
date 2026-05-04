/**
 * @file    thread_manager_v2.h
 * @brief   线程管理器（重构版）
 * 
 * 负责所有 RT-Thread 线程的创建、启动和管理。
 * 线程优先级从高到低：
 *   运动控制 (10) > 数据发送 (12) > 传感器采集 (15) > LED心跳 (25)
 */

#ifndef __THREAD_MANAGER_V2_H__
#define __THREAD_MANAGER_V2_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 线程栈大小定义 ========================== */
#define STACK_LED              512
#define STACK_IMU              1024
#define STACK_DEPTH_SENSOR     1024
#define STACK_UART_RX          512
#define STACK_DATA_SEND        1024
#define STACK_ULTRASOUND       768
#define STACK_PH               768
#define STACK_WATER_QUALITY    512
#define STACK_MOTION_CTRL      2048

/* ========================== 线程优先级定义 ========================== */
#define PRIO_MOTION_CTRL       10
#define PRIO_DATA_SEND         12
#define PRIO_IMU               15
#define PRIO_DEPTH_SENSOR      15
#define PRIO_ULTRASOUND        16
#define PRIO_PH                16
#define PRIO_WATER_QUALITY     16
#define PRIO_UART_RX           18
#define PRIO_LED               25

/* ========================== API ========================== */

/**
 * @brief 创建并启动所有系统线程
 *        在 system_state_init() 之后调用
 */
void thread_manager_start_all(void);

#ifdef __cplusplus
}
#endif

#endif /* __THREAD_MANAGER_V2_H__ */
