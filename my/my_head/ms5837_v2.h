/**
 * @file    ms5837_v2.h
 * @brief   MS5837-30BA 深度/水压传感器驱动（重构版）
 * 
 * 与原 MS5837.h 的区别：
 *   - 不再依赖全局 IIC_SCL_PIN/IIC_SDA_PIN
 *   - 所有操作通过 sw_i2c_bus_t 指针进行
 *   - 线程安全（由 sw_i2c_bus 内部的互斥锁保证）
 *   - 自动零位校准
 */

#ifndef __MS5837_V2_H__
#define __MS5837_V2_H__

#include "sw_i2c.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== MS5837 常量 ========================== */
#define MS5837_ADDR_WRITE   0xEC
#define MS5837_ADDR_READ    0xED
#define MS5837_CMD_RESET    0x1E
#define MS5837_CMD_PROM_RD  0xA0
#define MS5837_CMD_ADC_RD   0x00

/* 采样精度 */
#define MS5837_OSR_256   0x40
#define MS5837_OSR_512   0x42
#define MS5837_OSR_1024  0x44
#define MS5837_OSR_2048  0x46
#define MS5837_OSR_4096  0x48

/* 默认使用最高精度 */
#define MS5837_OSR_D1    MS5837_OSR_4096  /* 压力转换 */
#define MS5837_OSR_D2    0x58             /* 温度转换 8192 */

/* ========================== 数据结构 ========================== */

/** MS5837 校准系数 (从 PROM 读取) */
typedef struct {
    uint16_t c[7];  /* C0~C6，共7个16位系数 */
} ms5837_cal_t;

/** MS5837 传感器实例 */
typedef struct {
    sw_i2c_bus_t *bus;         /* I2C 总线 */
    ms5837_cal_t  cal;         /* 校准系数 */
    float         fluid_density; /* 液体密度 (淡水=1.0, 海水=1.025) */
    float         depth_offset;  /* 深度零位偏移 */
    uint8_t       initialized;   /* 初始化成功标志 */
} ms5837_dev_t;

/** MS5837 测量结果 */
typedef struct {
    float temperature;    /* 温度 (°C) */
    float pressure_mbar;  /* 压力 (毫巴) */
    float depth_m;        /* 深度 (米) */
} ms5837_result_t;

/* ========================== API ========================== */

/**
 * @brief 初始化 MS5837 传感器
 * @param dev       设备实例指针
 * @param bus       I2C 总线指针（&g_sys.i2c_bus_depth）
 * @param density   液体密度 (淡水1.0, 海水1.025)
 * @return 0=成功, 非0=失败
 */
uint8_t ms5837_init(ms5837_dev_t *dev, sw_i2c_bus_t *bus, float density);

/**
 * @brief 复位传感器
 */
void ms5837_reset(ms5837_dev_t *dev);

/**
 * @brief 读取 PROM 校准系数
 */
uint8_t ms5837_read_prom(ms5837_dev_t *dev);

/**
 * @brief CRC4 校验
 * @return 0=失败, 1=通过
 */
uint8_t ms5837_crc4(const ms5837_dev_t *dev);

/**
 * @brief 执行一次完整测量（压力 + 温度）
 * @param dev    设备实例
 * @param result 输出测量结果
 * @return 0=成功
 */
uint8_t ms5837_read(ms5837_dev_t *dev, ms5837_result_t *result);

/**
 * @brief 设置零位偏移（水面校准）
 */
static inline void ms5837_set_offset(ms5837_dev_t *dev, float offset)
{
    dev->depth_offset = offset;
}

#ifdef __cplusplus
}
#endif

#endif /* __MS5837_V2_H__ */
