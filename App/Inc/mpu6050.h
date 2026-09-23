#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* 初始化 MPU6050，并把陀螺仪量程配置为正负 2000 dps。 */
uint8_t MPU6050_Init(void);

/* 读取 WHO_AM_I 寄存器。正常 MPU6050 返回 0x68。 */
uint8_t MPU6050_ReadWhoAmI(uint8_t *who_am_i);

/* 返回初始化成功后实际使用的 7 位 I2C 地址：0x68 或 0x69。 */
uint8_t MPU6050_GetAddress(void);

/* 一次读取陀螺仪 X、Y、Z 三轴原始值。 */
uint8_t MPU6050_ReadGyroRaw(int16_t *gyro_x,
                            int16_t *gyro_y,
                            int16_t *gyro_z);

/* 静止时多次采样 Z 轴，求平均值并作为零偏返回。 */
uint8_t MPU6050_CalibrateGyroZ(uint16_t samples,
                               uint16_t sample_delay_ms,
                               float *bias_raw);

#endif
