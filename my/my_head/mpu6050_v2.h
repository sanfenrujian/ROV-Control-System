/**
 * @file    mpu6050_v2.h
 * @brief   MPU6050 驱动（重构版 - 使用 sw_i2c_bus）
 * 
 * 与原 mpu6050.h 的区别：
 *   - 不再依赖全局 IIC_SCL_PIN/IIC_SDA_PIN
 *   - 所有操作通过 sw_i2c_bus_t 指针进行
 *   - 线程安全（由 sw_i2c_bus 内部的互斥锁保证）
 */

#ifndef __MPU6050_V2_H__
#define __MPU6050_V2_H__

#include "sw_i2c.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== MPU6050 寄存器地址 ========================== */
#define MPU_ADDR                0x68    /* AD0=0 时的 I2C 地址 */
#define MPU_SMPLRT_DIV          0x19
#define MPU_CONFIG              0x1A
#define MPU_GYRO_CONFIG         0x1B
#define MPU_ACCEL_CONFIG        0x1C
#define MPU_FIFO_EN             0x23
#define MPU_INT_PIN_CFG         0x37
#define MPU_INT_ENABLE          0x38
#define MPU_ACCEL_XOUT_H        0x3B
#define MPU_TEMP_OUT_H          0x41
#define MPU_GYRO_XOUT_H         0x43
#define MPU_PWR_MGMT_1          0x6B
#define MPU_PWR_MGMT_2          0x6C
#define MPU_WHO_AM_I            0x75

/* ========================== API ========================== */

/**
 * @brief 初始化 MPU6050
 * @param bus  I2C 总线实例指针（通常为 &g_sys.i2c_bus_imu）
 * @return 0=成功, 非0=失败
 */
uint8_t mpu6050_init(sw_i2c_bus_t *bus);

/**
 * @brief 设置陀螺仪量程
 * @param fsr 0=±250, 1=±500, 2=±1000, 3=±2000 dps
 */
uint8_t mpu6050_set_gyro_fsr(sw_i2c_bus_t *bus, uint8_t fsr);

/**
 * @brief 设置加速度计量程
 * @param fsr 0=±2g, 1=±4g, 2=±8g, 3=±16g
 */
uint8_t mpu6050_set_accel_fsr(sw_i2c_bus_t *bus, uint8_t fsr);

/**
 * @brief 设置采样率
 * @param rate 采样率 Hz (4~1000)
 */
uint8_t mpu6050_set_rate(sw_i2c_bus_t *bus, uint16_t rate);

/**
 * @brief 读取加速度计原始值
 */
uint8_t mpu6050_get_accel(sw_i2c_bus_t *bus, int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief 读取陀螺仪原始值
 */
uint8_t mpu6050_get_gyro(sw_i2c_bus_t *bus, int16_t *gx, int16_t *gy, int16_t *gz);

/**
 * @brief 读取温度 (°C) * 100
 */
int16_t mpu6050_get_temp(sw_i2c_bus_t *bus);

/**
 * @brief 读取 WHO_AM_I 寄存器（应为 0x68）
 */
uint8_t mpu6050_get_id(sw_i2c_bus_t *bus);

/* ---------- DMP 相关（如使用 DMP 固件） ---------- */

/**
 * @brief 加载 DMP 固件并初始化
 * @note  需要链接 inv_mpu_dmp_motion_driver 库
 */
uint8_t mpu6050_dmp_init(sw_i2c_bus_t *bus);

/**
 * @brief 从 DMP 读取姿态角
 * @param pitch 俯仰角输出 (度)
 * @param roll  横滚角输出 (度)
 * @param yaw   偏航角输出 (度)
 * @return 0=成功
 */
uint8_t mpu6050_dmp_get_data(sw_i2c_bus_t *bus, float *pitch, float *roll, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_V2_H__ */
