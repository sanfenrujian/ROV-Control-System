/**
 * @file    ota.h
 * @brief   在线升级 (OTA) 模块
 *
 * 当前固件通过 UART 接收升级包，存储在 AXI SRAM 中，
 * 校验后通过 RAM 驻留的 Flash 编程程序写入内部 Flash。
 *
 * STM32H750VBTx Flash 布局：
 *   - 0x08000000 - 0x0801FFFF  (128KB, 1个扇区)
 *
 * OTA 流程：
 *   1. 上位机发送 OTA 握手命令
 *   2. 系统进入 OTA 模式，停止所有线程
 *   3. 上位机通过 UART2 发送固件 (分帧, 每帧 1024 字节)
 *   4. 固件存储到 AXI SRAM (0x24000000, 最大 112KB)
 *   5. 接收完毕后 CRC32 校验
 *   6. 调用 RAM 驻留的 flash_program() 擦除并写入 Flash
 *   7. 系统复位执行新固件
 *
 * ⚠️ 注意：H750 只有一个 128KB Flash 扇区，擦除时会擦除全部代码。
 *   因此 Flash 编程函数必须提前复制到 DTCM RAM 中执行。
 */

#ifndef __OTA_H__
#define __OTA_H__

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 常量定义 ========================== */

#define OTA_MAX_FIRMWARE_SIZE   (112 * 1024)   /* 最大固件大小 (112KB, 留16KB给Boot stub) */
#define OTA_BUF_ADDR            (0x24000000UL) /* AXI SRAM 起始地址作为固件缓冲区 */
#define OTA_FRAME_DATA_SIZE     1024           /* 每帧数据大小 */
#define OTA_FLASH_START         (0x08000000UL) /* Flash 起始地址 */
#define OTA_FLASH_END           (0x0801FFFFUL) /* Flash 结束地址 */
#define OTA_SECTOR_SIZE         (128 * 1024)   /* H750 扇区大小 */

/** OTA 帧头定义 */
#define OTA_SYNC_BYTE1          0xAA
#define OTA_SYNC_BYTE2          0x55
#define OTA_CMD_START           0x01            /* 开始传输: 包含固件总大小 + CRC32 */
#define OTA_CMD_DATA            0x02            /* 数据帧 */
#define OTA_CMD_END             0x03            /* 结束帧: 含总 CRC32 校验 */
#define OTA_CMD_ABORT           0x04            /* 取消传输 */
#define OTA_CMD_VERSION         0x05            /* 查询固件版本 */

/** OTA 应答 */
#define OTA_ACK_OK              0x00
#define OTA_ACK_ERR_CRC         0x01
#define OTA_ACK_ERR_SIZE        0x02
#define OTA_ACK_ERR_BUSY        0x03
#define OTA_ACK_ERR_FRAME       0x04
#define OTA_ACK_ERR_FLASH       0x05

/** OTA 状态 */
typedef enum {
    OTA_STATE_IDLE      = 0,    /* 空闲 */
    OTA_STATE_RECEIVING = 1,    /* 接收中 */
    OTA_STATE_VERIFYING = 2,    /* 校验中 */
    OTA_STATE_FLASHING  = 3,    /* 写入 Flash */
    OTA_STATE_COMPLETE  = 4,    /* 完成 */
    OTA_STATE_ERROR     = 5,    /* 错误 */
} ota_state_t;

/** OTA 帧结构 */
typedef struct __attribute__((packed)) {
    uint8_t  sync1;             /* 0xAA */
    uint8_t  sync2;             /* 0x55 */
    uint8_t  cmd;               /* 命令 */
    uint16_t len;               /* 数据长度 */
    uint32_t crc32;             /* 本帧 CRC32 */
    uint8_t  data[OTA_FRAME_DATA_SIZE]; /* 帧数据 */
} ota_frame_t;

/** OTA 上下文 */
typedef struct {
    volatile ota_state_t state;     /* 当前状态 */
    uint32_t total_size;            /* 固件总大小 (字节) */
    uint32_t received_size;         /* 已接收大小 */
    uint32_t expected_crc;          /* 期望的总 CRC32 */
    uint32_t computed_crc;          /* 计算的总 CRC32 */
    uint32_t frame_count;           /* 已接收帧数 */
    uint8_t  *buffer;               /* 固件缓冲区指针 (AXI SRAM) */
    uint32_t baudrate;              /* OTA 通信波特率 */
    void     (*progress_cb)(uint8_t percent); /* 进度回调 */
} ota_ctx_t;

/* ========================== API ========================== */

/**
 * @brief  初始化 OTA 模块
 * @param  baudrate   OTA 通信波特率 (默认 115200)
 */
void ota_init(uint32_t baudrate);

/**
 * @brief  设置进度回调
 */
void ota_set_progress_callback(void (*cb)(uint8_t percent));

/**
 * @brief  启动 OTA 升级
 *         停止所有线程，准备接收固件
 * @return 0=成功
 */
int ota_start(void);

/**
 * @brief  处理一帧 OTA 数据 (在 UART 回调中调用)
 * @param  data      接收到的帧数据
 * @param  len       帧长度
 * @return 应答码 (OTA_ACK_OK 等)
 */
uint8_t ota_process_frame(const uint8_t *data, uint16_t len);

/**
 * @brief  获取当前 OTA 状态
 */
ota_state_t ota_get_state(void);

/**
 * @brief  获取 OTA 进度百分比 (0-100)
 */
uint8_t ota_get_progress(void);

/**
 * @brief  取消 OTA
 */
void ota_abort(void);

/**
 * @brief  Flash 编程函数 (必须从 RAM 执行)
 *         擦除整个 Flash 扇区并写入新固件
 *         在调用前必须将本函数复制到 DTCM RAM
 * @param  src    源地址 (AXI SRAM 中的固件数据)
 * @param  dst    目标地址 (0x08000000)
 * @param  size   固件大小
 * @return 0=成功, 非0=失败
 */
int ota_flash_program(uint32_t src, uint32_t dst, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_H__ */
