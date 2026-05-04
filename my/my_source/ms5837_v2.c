/**
 * @file    ms5837_v2.c
 * @brief   MS5837-30BA 驱动实现（重构版）
 * 
 * 基于 BlueRobots 开源驱动重写，适配 sw_i2c_bus 架构。
 */

#include "ms5837_v2.h"
#include <rtthread.h>
#include <string.h>

/* ========================== 内部函数 ========================== */

/**
 * 发送 I2C 命令（无数据阶段）
 */
static void ms5837_send_cmd(ms5837_dev_t *dev, uint8_t cmd)
{
    sw_i2c_lock(dev->bus);

    sw_i2c_start(dev->bus);
    sw_i2c_send_byte(dev->bus, MS5837_ADDR_WRITE);
    sw_i2c_send_byte(dev->bus, cmd);
    sw_i2c_stop(dev->bus);

    sw_i2c_unlock(dev->bus);
}

/**
 * 读取 ADC 转换结果 (24-bit)
 */
static uint32_t ms5837_read_adc(ms5837_dev_t *dev, uint8_t cmd)
{
    uint8_t  buf[3];
    uint32_t result = 0;

    sw_i2c_lock(dev->bus);

    /* 发送转换命令 */
    sw_i2c_start(dev->bus);
    sw_i2c_send_byte(dev->bus, MS5837_ADDR_WRITE);
    sw_i2c_send_byte(dev->bus, cmd);
    sw_i2c_stop(dev->bus);

    sw_i2c_unlock(dev->bus);

    /* 等待转换完成 */
    rt_thread_mdelay(20);

    sw_i2c_lock(dev->bus);

    /* 读取结果 */
    sw_i2c_start(dev->bus);
    sw_i2c_send_byte(dev->bus, MS5837_ADDR_WRITE);
    sw_i2c_send_byte(dev->bus, MS5837_CMD_ADC_RD);
    sw_i2c_stop(dev->bus);

    sw_i2c_start(dev->bus);
    sw_i2c_send_byte(dev->bus, MS5837_ADDR_READ);
    buf[0] = sw_i2c_read_byte(dev->bus, 1); /* ACK */
    buf[1] = sw_i2c_read_byte(dev->bus, 1); /* ACK */
    buf[2] = sw_i2c_read_byte(dev->bus, 0); /* NACK */
    sw_i2c_stop(dev->bus);

    sw_i2c_unlock(dev->bus);

    result = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    return result;
}

/* ========================== 公开 API ========================== */

uint8_t ms5837_init(ms5837_dev_t *dev, sw_i2c_bus_t *bus, float density)
{
    if (dev == NULL || bus == NULL) return 1;

    memset(dev, 0, sizeof(ms5837_dev_t));
    dev->bus           = bus;
    dev->fluid_density = (density > 0.0f) ? density : 1.0f;
    dev->depth_offset  = 0.0f;

    /* 复位 */
    ms5837_reset(dev);
    rt_thread_mdelay(20);

    /* 读取 PROM */
    if (ms5837_read_prom(dev) != 0)
        return 1;

    /* CRC 校验 */
    if (!ms5837_crc4(dev))
    {
        rt_kprintf("[MS5837] CRC check failed!\r\n");
        return 1;
    }

    dev->initialized = 1;
    rt_kprintf("[MS5837] Init OK\r\n");
    return 0;
}

void ms5837_reset(ms5837_dev_t *dev)
{
    if (dev == NULL || dev->bus == NULL) return;
    ms5837_send_cmd(dev, MS5837_CMD_RESET);
}

uint8_t ms5837_read_prom(ms5837_dev_t *dev)
{
    if (dev == NULL || dev->bus == NULL) return 1;

    for (int i = 0; i < 7; i++)
    {
        uint8_t inth, intl;

        sw_i2c_lock(dev->bus);

        sw_i2c_start(dev->bus);
        sw_i2c_send_byte(dev->bus, MS5837_ADDR_WRITE);
        sw_i2c_send_byte(dev->bus, MS5837_CMD_PROM_RD + (i * 2));
        sw_i2c_stop(dev->bus);

        sw_i2c_unlock(dev->bus);
        rt_thread_mdelay(2);

        sw_i2c_lock(dev->bus);

        sw_i2c_start(dev->bus);
        sw_i2c_send_byte(dev->bus, MS5837_ADDR_READ);
        inth = sw_i2c_read_byte(dev->bus, 1);
        intl = sw_i2c_read_byte(dev->bus, 0);
        sw_i2c_stop(dev->bus);

        sw_i2c_unlock(dev->bus);

        dev->cal.c[i] = ((uint16_t)inth << 8) | intl;
    }

    return 0;
}

uint8_t ms5837_crc4(const ms5837_dev_t *dev)
{
    if (dev == NULL) return 0;

    uint16_t n_rem  = 0;
    uint16_t crc_read = dev->cal.c[0] >> 12;  /* 高4位是CRC */

    /* 清除 CRC 位 */
    uint16_t c0_cleaned = dev->cal.c[0] & 0x0FFF;

    uint16_t prom[8];
    prom[0] = c0_cleaned;
    for (int i = 1; i < 7; i++)
        prom[i] = dev->cal.c[i];
    prom[7] = 0;

    for (int i = 0; i < 16; i++)
    {
        if (i % 2 == 0)
            n_rem ^= prom[i >> 1];
        else
            n_rem ^= prom[i >> 1] & 0x00FF;

        for (int j = 0; j < 8; j++)
        {
            if (n_rem & 0x8000)
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }

    n_rem = (n_rem >> 12) & 0x000F;
    return (n_rem == crc_read) ? 1 : 0;
}

uint8_t ms5837_read(ms5837_dev_t *dev, ms5837_result_t *result)
{
    if (dev == NULL || !dev->initialized || result == NULL)
        return 1;

    /* 读取温度 ADC */
    uint32_t D2 = ms5837_read_adc(dev, MS5837_OSR_D2);
    rt_thread_mdelay(20);

    /* 读取压力 ADC */
    uint32_t D1 = ms5837_read_adc(dev, MS5837_OSR_D1);
    rt_thread_mdelay(20);

    /* --- 一阶温度补偿 --- */
    int32_t  dT;
    int64_t  OFF, SENS;
    int32_t  TEMP;

    const uint16_t *C = dev->cal.c;

    if (D2 > ((uint32_t)C[5] * 256))
    {
        dT   = (int32_t)(D2 - ((uint32_t)C[5] * 256));
        TEMP = 2000 + (int32_t)((int64_t)dT * C[6] / 8388608);
        OFF  = (int64_t)C[2] * 65536 + ((int64_t)C[4] * dT) / 128;
        SENS = (int64_t)C[1] * 32768 + ((int64_t)C[3] * dT) / 256;
    }
    else
    {
        dT   = (int32_t)(((uint32_t)C[5] * 256) - D2);
        TEMP = 2000 - (int32_t)((int64_t)dT * C[6] / 8388608);
        OFF  = (int64_t)C[2] * 65536 - ((int64_t)C[4] * dT) / 128;
        SENS = (int64_t)C[1] * 32768 - ((int64_t)C[3] * dT) / 256;
    }

    /* --- 二阶温度补偿 --- */
    int64_t OFF2 = 0, SENS2 = 0, T2 = 0;

    if (TEMP < 2000)
    {
        T2    = 3 * (int64_t)dT * dT / 8589934592LL;
        OFF2  = 3 * (TEMP - 2000) * (TEMP - 2000) / 2;
        SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / 8;

        if (TEMP < -1500)
        {
            OFF2  += 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 4 * (TEMP + 1500) * (TEMP + 1500);
        }
    }
    else
    {
        T2    = 2 * (int64_t)dT * dT / 137438953472LL;
        OFF2  = (TEMP - 2000) * (TEMP - 2000) / 16;
        SENS2 = 0;
    }

    OFF  -= OFF2;
    SENS -= SENS2;
    TEMP  = TEMP - (int32_t)T2;

    /* --- 计算压力和深度 --- */
    int32_t pressure_raw = (int32_t)(((int64_t)D1 * SENS / 2097152) - OFF) / 8192;

    result->temperature   = TEMP / 100.0f;
    result->pressure_mbar = pressure_raw / 10.0f;

    /* 深度 = 压力 / (密度 * 重力加速度) ，压力单位需要转为 Pa */
    /* pressure_raw 单位是 0.1mbar，即 10Pa */
    float pressure_pa = pressure_raw * 10.0f;
    result->depth_m = (pressure_pa / (dev->fluid_density * 9.80665f * 1000.0f))
                      - dev->depth_offset;

    return 0;
}
