/**
 * @file    uart_manager.h
 * @brief   多路 UART 管理模块
 * 
 * 解决原项目中多路 UART 共用一个 data 变量导致的致命并发问题。
 * 每路 UART 拥有独立的接收缓冲区和状态机。
 */

#ifndef __UART_MANAGER_H__
#define __UART_MANAGER_H__

#include "stm32h7xx_hal.h"
#include "system_state.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== UART 通道枚举 ========================== */

typedef enum {
    UART_CH_USART1 = 0,  /* USART1: 遥控器数据 */
    UART_CH_UART4  = 1,  /* UART4:  超声波 / 传感器 */
    UART_CH_UART5  = 2,  /* UART5:  深度传感器原始数据 */
    UART_CH_UART7  = 3,  /* UART7:  GPS 数据 */
    UART_CH_COUNT
} uart_channel_t;

/* ========================== 协议帧解析回调类型 ========================== */

/** 协议解析完成回调 */
typedef void (*uart_parser_cb_t)(const uint8_t *frame, uint16_t len);

/* ========================== API ========================== */

/**
 * @brief 初始化所有 UART 通道
 *        自动为每个通道分配独立缓冲区并启动中断接收
 */
void uart_manager_init(void);

/**
 * @brief HAL UART 中断回调入口（替换原 HAL_UART_RxCpltCallback 中的逻辑）
 * @param huart HAL UART 句柄
 */
void uart_rx_callback(UART_HandleTypeDef *huart);

/**
 * @brief 注册指定通道的协议解析回调
 * @param ch  UART 通道
 * @param cb  解析回调函数
 */
void uart_set_parser(uart_channel_t ch, uart_parser_cb_t cb);

/**
 * @brief 获取指定通道的最新协议帧（供线程消费）
 * @param ch    UART 通道
 * @param frame 输出帧数据指针
 * @param len   输出帧长度
 * @return 1=有新帧, 0=无新帧
 */
uint8_t uart_get_frame(uart_channel_t ch, const uint8_t **frame, uint16_t *len);

/* ========================== 协议解析函数（各通道独立实现） ========================== */

/** USART1 遥控器协议解析 (AA + ID + LEN + DATA + SUM) */
void uart_parse_remote(const uint8_t *frame, uint16_t len);

/** UART4 超声波/传感器协议解析 */
void uart_parse_ultrasound(const uint8_t *frame, uint16_t len);

/** UART5 深度传感器原始数据解析 */
void uart_parse_depth(const uint8_t *frame, uint16_t len);

/** UART7 GPS NMEA 数据解析 */
void uart_parse_gps(const uint8_t *frame, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __UART_MANAGER_H__ */
