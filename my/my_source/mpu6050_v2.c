/**
 * @file    mpu6050_v2.c
 * @brief   MPU6050 驱动实现（重构版）
 */

#include "mpu6050_v2.h"
#include <rtthread.h>

/* ========================== 内部辅助 ========================== */

/**
 * 带锁的寄存器写入
 */
static uint8_t reg_write(sw_i2c_bus_t *bus, uint8_t reg, uint8_t data)
{
    return sw_i2c_write_reg(bus, MPU_ADDR, reg, &data, 1);
}

/**
 * 带锁的寄存器读取
 */
static uint8_t reg_read(sw_i2c_bus_t *bus, uint8_t reg, uint8_t *data, uint8_t len)
{
    return sw_i2c_read_reg(bus, MPU_ADDR, reg, data, len);
}

/* ========================== 公开 API ========================== */

uint8_t mpu6050_init(sw_i2c_bus_t *bus)
{
    if (bus == NULL) return 1;

    uint8_t id;

    /* 复位设备 */
    reg_write(bus, MPU_PWR_MGMT_1, 0x80);
    rt_thread_mdelay(100);

    /* 唤醒 */
    reg_write(bus, MPU_PWR_MGMT_1, 0x00);

    /* 检查设备 ID */
    id = mpu6050_get_id(bus);
    if (id != MPU_ADDR)
        return 1; /* 设备不存在 */

    /* 配置传感器 */
    mpu6050_set_gyro_fsr(bus, 3);   /* ±2000 dps */
    mpu6050_set_accel_fsr(bus, 0);  /* ±2g */
    mpu6050_set_rate(bus, 50);      /* 50Hz */

    /* 关闭中断、FIFO、I2C主模式 */
    reg_write(bus, MPU_INT_ENABLE, 0x00);
    reg_write(bus, 0x6A /* USER_CTRL */, 0x00);
    reg_write(bus, MPU_FIFO_EN, 0x00);

    /* INT 引脚低电平有效 */
    reg_write(bus, MPU_INT_PIN_CFG, 0x80);

    /* 时钟源：PLL X轴陀螺仪 */
    reg_write(bus, MPU_PWR_MGMT_1, 0x01);
    reg_write(bus, MPU_PWR_MGMT_2, 0x00);

    return 0;
}

uint8_t mpu6050_set_gyro_fsr(sw_i2c_bus_t *bus, uint8_t fsr)
{
    if (fsr > 3) fsr = 3;
    return reg_write(bus, MPU_GYRO_CONFIG, fsr << 3);
}

uint8_t mpu6050_set_accel_fsr(sw_i2c_bus_t *bus, uint8_t fsr)
{
    if (fsr > 3) fsr = 3;
    return reg_write(bus, MPU_ACCEL_CONFIG, fsr << 3);
}

uint8_t mpu6050_set_rate(sw_i2c_bus_t *bus, uint16_t rate)
{
    if (rate > 1000) rate = 1000;
    if (rate < 4)    rate = 4;

    uint8_t div = 1000 / rate - 1;
    uint8_t ret = reg_write(bus, MPU_SMPLRT_DIV, div);

    /* 自动设置 DLPF 为采样率的一半 */
    uint8_t dlpf = 6; /* 默认5Hz */
    if (rate >= 188)      dlpf = 1;
    else if (rate >= 98)  dlpf = 2;
    else if (rate >= 42)  dlpf = 3;
    else if (rate >= 20)  dlpf = 4;
    else if (rate >= 10)  dlpf = 5;

    reg_write(bus, MPU_CONFIG, dlpf);

    return ret;
}

uint8_t mpu6050_get_id(sw_i2c_bus_t *bus)
{
    uint8_t id = 0;
    reg_read(bus, MPU_WHO_AM_I, &id, 1);
    return id;
}

uint8_t mpu6050_get_accel(sw_i2c_bus_t *bus, int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    if (reg_read(bus, MPU_ACCEL_XOUT_H, buf, 6) != 0)
        return 1;

    if (ax) *ax = ((int16_t)buf[0] << 8) | buf[1];
    if (ay) *ay = ((int16_t)buf[2] << 8) | buf[3];
    if (az) *az = ((int16_t)buf[4] << 8) | buf[5];

    return 0;
}

uint8_t mpu6050_get_gyro(sw_i2c_bus_t *bus, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    if (reg_read(bus, MPU_GYRO_XOUT_H, buf, 6) != 0)
        return 1;

    if (gx) *gx = ((int16_t)buf[0] << 8) | buf[1];
    if (gy) *gy = ((int16_t)buf[2] << 8) | buf[3];
    if (gz) *gz = ((int16_t)buf[4] << 8) | buf[5];

    return 0;
}

int16_t mpu6050_get_temp(sw_i2c_bus_t *bus)
{
    uint8_t buf[2];
    if (reg_read(bus, MPU_TEMP_OUT_H, buf, 2) != 0)
        return 0;

    int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
    /* 公式: °C = raw/340 + 36.53，返回 *100 */
    return (int16_t)(36.53f + (float)raw / 340.0f) * 100;
}

/* ========================== DMP 相关（占位） ========================== */

uint8_t mpu6050_dmp_init(sw_i2c_bus_t *bus)
{
    /*
     * DMP 初始化需要加载固件镜像（inv_mpu_dmp_motion_driver 库）。
     * 此处为占位，需根据实际使用的 DMP 库进行适配。
     * 
     * 注意：库内部的 I2C 读写需要适配为 sw_i2c_bus 系列函数。
     */
    (void)bus;
    return 0;
}

uint8_t mpu6050_dmp_get_data(sw_i2c_bus_t *bus, float *pitch, float *roll, float *yaw)
{
    /*
     * 从 DMP FIFO 读取四元数并转换为欧拉角。
     * 此处为占位，需根据实际 DMP 库适配。
     */
    (void)bus;
    if (pitch) *pitch = 0.0f;
    if (roll)  *roll  = 0.0f;
    if (yaw)   *yaw   = 0.0f;
    return 0;
}
