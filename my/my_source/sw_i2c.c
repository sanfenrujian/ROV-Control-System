/**
 * @file    sw_i2c.c
 * @brief   Software I2C Bus Implementation
 * 
 * 基于 GPIO 位操作的软件 I2C 总线驱动。
 * 每个总线实例（sw_i2c_bus_t）完全独立，通过 RT-Thread 互斥锁
 * 保证多线程环境下的独占访问，彻底解决多设备共享 I2C 的并发问题。
 */

#include "sw_i2c.h"
#include <string.h>

/* ========================== 内部辅助宏 ========================== */

/** 根据总线实例获取 SCL/SDA 的 GPIO 操作 */
#define BUS_SCL_H(bus)  HAL_GPIO_WritePin((bus)->gpio_port, (bus)->scl_pin, GPIO_PIN_SET)
#define BUS_SCL_L(bus)  HAL_GPIO_WritePin((bus)->gpio_port, (bus)->scl_pin, GPIO_PIN_RESET)
#define BUS_SDA_H(bus)  HAL_GPIO_WritePin((bus)->gpio_port, (bus)->sda_pin, GPIO_PIN_SET)
#define BUS_SDA_L(bus)  HAL_GPIO_WritePin((bus)->gpio_port, (bus)->sda_pin, GPIO_PIN_RESET)
#define BUS_SDA_RD(bus) HAL_GPIO_ReadPin((bus)->gpio_port, (bus)->sda_pin)

/** 微秒级延时（使用 DWT 或简单循环，这里使用 RT-Thread 的精确延时） */
#define I2C_DELAY_US(bus) do { \
    if ((bus)->delay_us > 0) { \
        volatile uint32_t _cnt = (bus)->delay_us * (SystemCoreClock / 1000000 / 4); \
        while (_cnt--) { __NOP(); } \
    } \
} while(0)

/* ========================== GPIO 初始化 ========================== */

static void i2c_gpio_init(sw_i2c_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能 GPIO 时钟 */
    if (bus->gpio_port == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (bus->gpio_port == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (bus->gpio_port == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (bus->gpio_port == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (bus->gpio_port == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (bus->gpio_port == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();

    /* SCL 和 SDA 都配置为开漏输出（模拟 I2C 总线特性） */
    GPIO_InitStruct.Pin   = bus->scl_pin | bus->sda_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->gpio_port, &GPIO_InitStruct);

    /* 初始状态：总线空闲（SCL=1, SDA=1） */
    BUS_SCL_H(bus);
    BUS_SDA_H(bus);
}

/* ========================== 公开 API ========================== */

rt_err_t sw_i2c_init(sw_i2c_bus_t *bus,
                     GPIO_TypeDef  *gpio_port,
                     uint16_t       scl_pin,
                     uint16_t       sda_pin,
                     uint32_t       freq_hz,
                     const char    *name)
{
    if (bus == NULL || gpio_port == NULL || name == NULL)
        return RT_ERROR;

    memset(bus, 0, sizeof(sw_i2c_bus_t));

    bus->gpio_port = gpio_port;
    bus->scl_pin   = scl_pin;
    bus->sda_pin   = sda_pin;
    strncpy(bus->name, name, RT_NAME_MAX - 1);
    bus->name[RT_NAME_MAX - 1] = '\0';

    /* 计算半周期延时 */
    if (freq_hz == 0) freq_hz = 100000;
    bus->delay_us   = 500000 / freq_hz;  /* 半周期 = 1/(2*freq) 换算为微秒 */
    if (bus->delay_us < 1) bus->delay_us = 1;

    bus->timeout_ms = 50; /* ACK 超时默认 50ms */

    /* GPIO 初始化 */
    i2c_gpio_init(bus);

    /* 创建互斥锁 */
    bus->mutex = rt_mutex_create(name, RT_IPC_FLAG_PRIO);
    if (bus->mutex == RT_NULL)
        return RT_ERROR;

    return RT_EOK;
}

rt_err_t sw_i2c_deinit(sw_i2c_bus_t *bus)
{
    if (bus == NULL) return RT_ERROR;

    if (bus->mutex != RT_NULL)
    {
        rt_mutex_delete(bus->mutex);
        bus->mutex = RT_NULL;
    }
    return RT_EOK;
}

/* ========================== I2C 基本时序 ========================== */

void sw_i2c_start(sw_i2c_bus_t *bus)
{
    /* SCL 高电平期间 SDA 下降沿 = 起始信号 */
    BUS_SDA_H(bus);
    I2C_DELAY_US(bus);
    BUS_SCL_H(bus);
    I2C_DELAY_US(bus);
    BUS_SDA_L(bus);
    I2C_DELAY_US(bus);
    BUS_SCL_L(bus);
    I2C_DELAY_US(bus);
}

void sw_i2c_stop(sw_i2c_bus_t *bus)
{
    /* SCL 高电平期间 SDA 上升沿 = 停止信号 */
    BUS_SDA_L(bus);
    I2C_DELAY_US(bus);
    BUS_SCL_H(bus);
    I2C_DELAY_US(bus);
    BUS_SDA_H(bus);
    I2C_DELAY_US(bus);
}

uint8_t sw_i2c_send_byte(sw_i2c_bus_t *bus, uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & 0x80)
            BUS_SDA_H(bus);
        else
            BUS_SDA_L(bus);

        I2C_DELAY_US(bus);
        BUS_SCL_H(bus);
        I2C_DELAY_US(bus);
        BUS_SCL_L(bus);
        I2C_DELAY_US(bus);

        data <<= 1;
    }

    /* 释放 SDA 以接收 ACK */
    BUS_SDA_H(bus);
    I2C_DELAY_US(bus);

    return sw_i2c_wait_ack(bus);
}

uint8_t sw_i2c_read_byte(sw_i2c_bus_t *bus, uint8_t ack)
{
    uint8_t value = 0;

    BUS_SDA_H(bus); /* 释放 SDA */
    I2C_DELAY_US(bus);

    for (uint8_t i = 0; i < 8; i++)
    {
        value <<= 1;

        BUS_SCL_H(bus);
        I2C_DELAY_US(bus);

        if (BUS_SDA_RD(bus))
            value |= 0x01;

        BUS_SCL_L(bus);
        I2C_DELAY_US(bus);
    }

    /* 发送 ACK/NACK */
    if (ack)
        sw_i2c_send_ack(bus);
    else
        sw_i2c_send_nack(bus);

    return value;
}

uint8_t sw_i2c_wait_ack(sw_i2c_bus_t *bus)
{
    uint32_t timeout = bus->timeout_ms * 1000; /* 转换为微秒近似值 */
    uint8_t  ret     = 1; /* 默认失败 */

    BUS_SCL_H(bus);
    I2C_DELAY_US(bus);

    while (timeout--)
    {
        if (BUS_SDA_RD(bus) == GPIO_PIN_RESET)
        {
            ret = 0; /* 收到 ACK */
            break;
        }
        /* 简单轮询等待（生产环境可用 rt_thread_delay 配合信号量优化） */
        for (volatile int _d = 0; _d < 10; _d++) { __NOP(); }
    }

    BUS_SCL_L(bus);
    I2C_DELAY_US(bus);

    return ret;
}

void sw_i2c_send_ack(sw_i2c_bus_t *bus)
{
    BUS_SDA_L(bus);
    I2C_DELAY_US(bus);
    BUS_SCL_H(bus);
    I2C_DELAY_US(bus);
    BUS_SCL_L(bus);
    I2C_DELAY_US(bus);
    BUS_SDA_H(bus);
}

void sw_i2c_send_nack(sw_i2c_bus_t *bus)
{
    BUS_SDA_H(bus);
    I2C_DELAY_US(bus);
    BUS_SCL_H(bus);
    I2C_DELAY_US(bus);
    BUS_SCL_L(bus);
    I2C_DELAY_US(bus);
}

/* ========================== 高级操作 ========================== */

uint8_t sw_i2c_check_device(sw_i2c_bus_t *bus, uint8_t addr)
{
    uint8_t ack;

    sw_i2c_lock(bus);
    sw_i2c_start(bus);
    ack = sw_i2c_send_byte(bus, (addr << 1) | 0); /* 写方向 */
    sw_i2c_stop(bus);
    sw_i2c_unlock(bus);

    return ack;
}

uint8_t sw_i2c_write_reg(sw_i2c_bus_t *bus, uint8_t addr,
                         uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t ret;

    sw_i2c_lock(bus);

    sw_i2c_start(bus);
    ret = sw_i2c_send_byte(bus, (addr << 1) | 0); /* 写地址 */
    if (ret != 0) goto exit;

    ret = sw_i2c_send_byte(bus, reg); /* 写寄存器 */
    if (ret != 0) goto exit;

    for (uint8_t i = 0; i < len; i++)
    {
        ret = sw_i2c_send_byte(bus, data[i]);
        if (ret != 0) goto exit;
    }

exit:
    sw_i2c_stop(bus);
    sw_i2c_unlock(bus);
    return ret;
}

uint8_t sw_i2c_read_reg(sw_i2c_bus_t *bus, uint8_t addr,
                        uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t ret;

    sw_i2c_lock(bus);

    /* 先写寄存器地址 */
    sw_i2c_start(bus);
    ret = sw_i2c_send_byte(bus, (addr << 1) | 0);
    if (ret != 0) goto exit;
    ret = sw_i2c_send_byte(bus, reg);
    if (ret != 0) goto exit;

    /* 重复起始 + 读 */
    sw_i2c_start(bus);
    ret = sw_i2c_send_byte(bus, (addr << 1) | 1);
    if (ret != 0) goto exit;

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t ack = (i < len - 1) ? 1 : 0;
        data[i] = sw_i2c_read_byte(bus, ack);
    }

    ret = 0;

exit:
    sw_i2c_stop(bus);
    sw_i2c_unlock(bus);
    return ret;
}
