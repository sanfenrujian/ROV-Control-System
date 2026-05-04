/**
 * @file    sw_i2c.h
 * @brief   Software I2C Bus Abstraction with Mutex Protection
 * 
 * 解决原项目中多个 I2C 设备（MPU6050、MS5837）共用全局引脚变量、
 * 运行时动态切换引脚无保护导致的并发安全问题。
 * 
 * 设计方案：
 *   每个 I2C 设备拥有独立的 sw_i2c_bus_t 实例，
 *   每个实例封装了 GPIO 端口/引脚 + RT-Thread 互斥锁。
 *   总线操作前自动获取锁，操作后释放，保证多线程安全。
 */

#ifndef __SW_I2C_H__
#define __SW_I2C_H__

#include "stm32h7xx_hal.h"
#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 类型定义 ========================== */

/** 软件 I2C 总线实例 */
typedef struct {
    /* --- 硬件配置 --- */
    GPIO_TypeDef *gpio_port;    /* SCL/SDA 所在 GPIO 端口 */
    uint16_t      scl_pin;      /* SCL 引脚号 */
    uint16_t      sda_pin;      /* SDA 引脚号 */

    /* --- 时序参数 --- */
    uint32_t      delay_us;     /* 半周期延迟（微秒），标准模式 ~5us，快速模式 ~1.25us */
    uint32_t      timeout_ms;   /* 等待 ACK 超时（毫秒） */

    /* --- RTOS 同步 --- */
    rt_mutex_t    mutex;        /* 互斥锁，保证多线程独占访问 */
    char          name[RT_NAME_MAX]; /* 总线名称，用于调试 */
} sw_i2c_bus_t;

/* ========================== API ========================== */

/**
 * @brief  初始化软件 I2C 总线实例
 * @param  bus        总线实例指针
 * @param  gpio_port  GPIO 端口 (如 GPIOB)
 * @param  scl_pin    SCL 引脚 (如 GPIO_PIN_4)
 * @param  sda_pin    SDA 引脚 (如 GPIO_PIN_5)
 * @param  freq_hz    期望的 I2C 时钟频率 (如 100000 标准模式, 400000 快速模式)
 * @param  name       总线名称（用于互斥锁命名，最多7字符）
 * @return RT_EOK 成功，其他值失败
 */
rt_err_t sw_i2c_init(sw_i2c_bus_t *bus,
                     GPIO_TypeDef  *gpio_port,
                     uint16_t       scl_pin,
                     uint16_t       sda_pin,
                     uint32_t       freq_hz,
                     const char    *name);

/**
 * @brief  反初始化总线（释放互斥锁等资源）
 */
rt_err_t sw_i2c_deinit(sw_i2c_bus_t *bus);

/* ---------- 带锁的总线操作（推荐使用） ---------- */

/** 发送起始信号 */
void sw_i2c_start(sw_i2c_bus_t *bus);

/** 发送停止信号 */
void sw_i2c_stop(sw_i2c_bus_t *bus);

/** 发送一个字节，返回 0=收到ACK, 1=未收到ACK */
uint8_t sw_i2c_send_byte(sw_i2c_bus_t *bus, uint8_t data);

/** 读取一个字节，ack=1 发送ACK，ack=0 发送NACK */
uint8_t sw_i2c_read_byte(sw_i2c_bus_t *bus, uint8_t ack);

/** 等待 ACK，返回 0=收到ACK, 1=超时 */
uint8_t sw_i2c_wait_ack(sw_i2c_bus_t *bus);

/** 发送 ACK */
void sw_i2c_send_ack(sw_i2c_bus_t *bus);

/** 发送 NACK */
void sw_i2c_send_nack(sw_i2c_bus_t *bus);

/* ---------- 高级操作 ---------- */

/**
 * @brief  检测指定地址的设备是否存在
 * @return 0=存在, 非0=不存在
 */
uint8_t sw_i2c_check_device(sw_i2c_bus_t *bus, uint8_t addr);

/**
 * @brief  向设备寄存器写入数据
 * @param  bus    总线实例
 * @param  addr   7位设备地址
 * @param  reg    寄存器地址
 * @param  data   数据指针
 * @param  len    数据长度
 * @return 0=成功, 非0=失败
 */
uint8_t sw_i2c_write_reg(sw_i2c_bus_t *bus, uint8_t addr,
                         uint8_t reg, const uint8_t *data, uint8_t len);

/**
 * @brief  从设备寄存器读取数据
 * @return 0=成功, 非0=失败
 */
uint8_t sw_i2c_read_reg(sw_i2c_bus_t *bus, uint8_t addr,
                        uint8_t reg, uint8_t *data, uint8_t len);

/* ---------- 带锁保护的复合操作 ---------- */

/**
 * @brief  获取总线锁（配对 sw_i2c_unlock 使用，用于需要连续操作多个 I2C 原语的场景）
 *         注意：通常在 sw_i2c_start 前调用，sw_i2c_stop 后调用 sw_i2c_unlock
 */
static inline rt_err_t sw_i2c_lock(sw_i2c_bus_t *bus)
{
    return rt_mutex_take(bus->mutex, RT_WAITING_FOREVER);
}

static inline rt_err_t sw_i2c_unlock(sw_i2c_bus_t *bus)
{
    return rt_mutex_release(bus->mutex);
}

#ifdef __cplusplus
}
#endif

#endif /* __SW_I2C_H__ */
