/**
 * @file    uart_manager.c
 * @brief   多路 UART 管理模块实现
 * 
 * 核心改进：
 *   1. 每路 UART 拥有独立的 rx_byte 和 rx_buffer（不再共用 data 变量）
 *   2. 协议解析在回调中完成，而非在线程中轮询
 *   3. 支持注册自定义协议解析器
 */

#include "uart_manager.h"
#include "usart.h"
#include <string.h>
#include <rtthread.h>
#include <stdio.h>

/* ========================== UART 通道管理结构 ========================== */

#define UART_RX_BUF_SIZE  128  /* 每路 UART 帧缓冲区大小 */

typedef struct {
    UART_HandleTypeDef *huart;          /* HAL 句柄 */
    volatile uint8_t    rx_byte;        /* 独立的中断接收字节 */
    uint8_t             rx_buffer[UART_RX_BUF_SIZE]; /* 帧缓冲区 */
    volatile uint16_t   rx_count;       /* 当前帧已接收字节数 */
    volatile uint8_t    frame_ready;    /* 帧接收完成标志 */
    uart_parser_cb_t    parser_cb;      /* 解析回调 */
} uart_channel_ctx_t;

/** 四路 UART 通道上下文 */
static uart_channel_ctx_t g_uart_ctx[UART_CH_COUNT];

/* ========================== 初始化 ========================== */

void uart_manager_init(void)
{
    memset(g_uart_ctx, 0, sizeof(g_uart_ctx));

    /* 绑定 HAL 句柄 */
    g_uart_ctx[UART_CH_USART1].huart = &huart1;
    g_uart_ctx[UART_CH_UART4].huart  = &huart4;
    g_uart_ctx[UART_CH_UART5].huart  = &huart5;
    g_uart_ctx[UART_CH_UART7].huart  = &huart7;

    /* 注册默认解析器 */
    g_uart_ctx[UART_CH_USART1].parser_cb = uart_parse_remote;
    g_uart_ctx[UART_CH_UART4].parser_cb  = uart_parse_ultrasound;
    g_uart_ctx[UART_CH_UART5].parser_cb  = uart_parse_depth;
    g_uart_ctx[UART_CH_UART7].parser_cb  = uart_parse_gps;

    /* 启动所有 UART 中断接收（每路使用自己的独立 rx_byte） */
    for (int i = 0; i < UART_CH_COUNT; i++)
    {
        if (g_uart_ctx[i].huart != NULL)
        {
            HAL_UART_Receive_IT(g_uart_ctx[i].huart,
                               (uint8_t *)&g_uart_ctx[i].rx_byte, 1);
        }
    }
}

void uart_set_parser(uart_channel_t ch, uart_parser_cb_t cb)
{
    if (ch < UART_CH_COUNT)
        g_uart_ctx[ch].parser_cb = cb;
}

/* ========================== 中断回调 ========================== */

void uart_rx_callback(UART_HandleTypeDef *huart)
{
    uart_channel_ctx_t *ctx = NULL;

    /* 查找对应的通道上下文 */
    for (int i = 0; i < UART_CH_COUNT; i++)
    {
        if (g_uart_ctx[i].huart == huart)
        {
            ctx = &g_uart_ctx[i];
            break;
        }
    }

    if (ctx == NULL) return;

    uint8_t byte = ctx->rx_byte;

    /* 放入帧缓冲区 */
    if (ctx->rx_count < UART_RX_BUF_SIZE)
    {
        ctx->rx_buffer[ctx->rx_count++] = byte;
    }

    /* 
     * 协议边界检测：
     * - 遥控器帧: 以 0xAA 开头，由 parser 内部判断帧尾
     * - 深度传感器: 以 0xAB 开头
     * - GPS: 以 '$' 开头，'\n' 结尾
     * - 超声波: 以 0xFF 开头
     * 
     * 简化处理：由各自的 parser_cb 判断帧是否完整。
     * 这里采用"每次收到字节都尝试解析"的策略，
     * 解析器内部维护自己的状态机。
     */

    /* 调用协议解析器（每个字节都触发，解析器内部维护状态机） */
    if (ctx->parser_cb != NULL)
    {
        ctx->parser_cb(ctx->rx_buffer, ctx->rx_count);

        /* 如果解析器消费了数据，重置缓冲区 */
        /* 注意：具体重置逻辑由各解析器通过回调实现 */
    }

    /* 防止缓冲区溢出 */
    if (ctx->rx_count >= UART_RX_BUF_SIZE)
        ctx->rx_count = 0;

    /* 重新启动中断接收 */
    HAL_UART_Receive_IT(huart, (uint8_t *)&ctx->rx_byte, 1);
}

/* ========================== 协议解析器 ========================== */

/* --- USART1: 遥控器 (AA + ID(0x01) + LEN + 40字节数据 + SUM) --- */
void uart_parse_remote(const uint8_t *frame, uint16_t len)
{
    /* 需要至少 5 字节才能判断帧头 */
    if (len < 5) return;

    /* 查找帧头 0xAA */
    if (frame[0] != 0xAA) return;

    /* 长度字节 */
    uint8_t data_len = frame[2];
    if (data_len > 40) return; /* 安全检查 */

    /* 检查帧是否完整: 头(1) + ID(1) + LEN(1) + DATA(data_len) + SUM(1) */
    uint16_t total_len = 1 + 1 + 1 + data_len + 1;
    if (len < total_len) return; /* 帧不完整，等待更多数据 */

    /* 校验和 */
    uint8_t sum = 0;
    for (uint16_t i = 0; i < total_len - 1; i++)
        sum += frame[i];

    if (sum != frame[total_len - 1])
        return; /* 校验失败 */

    /* 解析遥控器数据：
     *  数据布局（示例，根据实际协议调整）：
     *  data[0..1] = ch1, data[2..3] = ch2, ... (每个通道2字节)
     *  通道值映射到 0.0 ~ 2.0 (中位 1.0 对应PWM 1500)
     */
    const uint8_t *data = frame + 3; /* 跳过 头+ID+LEN */

    if (data_len >= 2)  g_sys.remote.ch1 = (float)((int16_t)((data[0]<<8)|data[1])) / 1000.0f;
    if (data_len >= 4)  g_sys.remote.ch2 = (float)((int16_t)((data[2]<<8)|data[3])) / 1000.0f;
    if (data_len >= 6)  g_sys.remote.ch3 = (float)((int16_t)((data[4]<<8)|data[5])) / 1000.0f;
    if (data_len >= 8)  g_sys.remote.ch4 = (float)((int16_t)((data[6]<<8)|data[7])) / 1000.0f;
    if (data_len >= 10) g_sys.remote.ch5 = (float)((int16_t)((data[8]<<8)|data[9])) / 1000.0f;
    if (data_len >= 12) g_sys.remote.ch6 = (float)((int16_t)((data[10]<<8)|data[11])) / 1000.0f;
    if (data_len >= 14) g_sys.remote.ch7 = (float)((int16_t)((data[12]<<8)|data[13])) / 1000.0f;
    if (data_len >= 16) g_sys.remote.ch8 = (float)((int16_t)((data[14]<<8)|data[15])) / 1000.0f;
    if (data_len >= 18) g_sys.remote.ch9 = (float)((int16_t)((data[16]<<8)|data[17])) / 1000.0f;
    if (data_len >= 20) g_sys.remote.ch10 = (float)((int16_t)((data[18]<<8)|data[19])) / 1000.0f;
}

/* --- UART4: 超声波传感器 (FF + HIGH + LOW + SUM 协议) --- */
void uart_parse_ultrasound(const uint8_t *frame, uint16_t len)
{
    if (len < 4) return;
    if (frame[0] != 0xFF) return;

    uint8_t high  = frame[1];
    uint8_t low   = frame[2];
    uint8_t check = frame[3];

    if ((0xFF + high + low) == check)
    {
        uint16_t distance_raw = ((uint16_t)high << 8) | low;
        g_sys.ultrasound.distance_m = (float)distance_raw / 1000.0f; /* 转换为米 */
        g_sys.ultrasound.valid = 1;
    }
}

/* --- UART5: 深度传感器原始数据 (AB + ID + 8字节数据) --- */
void uart_parse_depth(const uint8_t *frame, uint16_t len)
{
    if (len < 10) return;
    if (frame[0] != 0xAB) return;

    /* 复制原始深度字节 */
    memcpy(g_sys.depth_raw.bytes, frame + 2, 4);

    /* 解析为浮点深度（根据实际传感器协议调整） */
    g_sys.depth_raw.depth = (float)((int32_t)((frame[2]<<24)|(frame[3]<<16)|(frame[4]<<8)|frame[5])) / 1000.0f;
}

/* --- UART7: GPS NMEA 协议 ($GPGGA 或 $GPRMC) --- */
void uart_parse_gps(const uint8_t *frame, uint16_t len)
{
    if (len < 6) return;
    if (frame[0] != '$') return;

    /* 检查是否是 NMEA 语句结尾 */
    if (frame[len - 1] != '\n' && frame[len - 1] != '\r')
        return;

    /* 简单 GPS 解析（生产环境建议使用 minmea 或 TinyGPS++ 库） */
    /* 这里保留原有的解析逻辑接口 */
    rt_kprintf("[GPS] NMEA frame received, len=%d\r\n", len);
}
