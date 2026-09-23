#include "mpu6050.h"
#include "app_config.h"
#include "bsp_i2c2.h"
#include "bsp_time.h"

#define MPU6050_REG_SMPLRT_DIV       0x19U
#define MPU6050_REG_CONFIG           0x1AU
#define MPU6050_REG_GYRO_CONFIG      0x1BU
#define MPU6050_REG_ACCEL_CONFIG     0x1CU
#define MPU6050_REG_GYRO_XOUT_H      0x43U
#define MPU6050_REG_PWR_MGMT_1       0x6BU
#define MPU6050_REG_WHO_AM_I         0x75U

#define MPU6050_EXPECTED_WHO_AM_I    0x68U
#define MPU6050_ALT_I2C_ADDRESS      0x69U

/* AD0 悬空时部分模块可能被识别成 0x69，所以初始化时两个地址都试一次。 */
static uint8_t g_mpu6050_address = MPU6050_I2C_ADDRESS;

static uint8_t MPU6050_ReadInt16(uint8_t high_reg, int16_t *value)
{
  uint8_t high;
  uint8_t low;

  if ((value == 0) ||
      (BSP_I2C2_ReadRegister(g_mpu6050_address, high_reg, &high) == 0U) ||
      (BSP_I2C2_ReadRegister(g_mpu6050_address,
                             (uint8_t)(high_reg + 1U),
                             &low) == 0U))
  {
    return 0U;
  }

  *value = (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
  return 1U;
}

uint8_t MPU6050_ReadWhoAmI(uint8_t *who_am_i)
{
  return BSP_I2C2_ReadRegister(g_mpu6050_address,
                               MPU6050_REG_WHO_AM_I,
                               who_am_i);
}

uint8_t MPU6050_GetAddress(void)
{
  return g_mpu6050_address;
}

static uint8_t MPU6050_CheckAddress(uint8_t address7)
{
  uint8_t who_am_i = 0U;

  if (BSP_I2C2_ReadRegister(address7,
                             MPU6050_REG_WHO_AM_I,
                             &who_am_i) == 0U)
  {
    return 0U;
  }
  if (who_am_i != MPU6050_EXPECTED_WHO_AM_I)
  {
    return 0U;
  }

  g_mpu6050_address = address7;
  return 1U;
}

uint8_t MPU6050_Init(void)
{
  uint8_t found = 0U;

  BSP_I2C2_Init();
  BSP_DelayMs(100U);

  g_mpu6050_address = MPU6050_I2C_ADDRESS;
  found = MPU6050_CheckAddress(g_mpu6050_address);
  if ((found == 0U) && (MPU6050_I2C_ADDRESS != MPU6050_ALT_I2C_ADDRESS))
  {
    BSP_DelayMs(2U);
    found = MPU6050_CheckAddress(MPU6050_ALT_I2C_ADDRESS);
  }
  if (found == 0U)
  {
    return 0U;
  }

  /* 复位设备，然后唤醒并选择陀螺仪 X 轴 PLL 作为时钟源。 */
  if (BSP_I2C2_WriteRegister(g_mpu6050_address,
                             MPU6050_REG_PWR_MGMT_1,
                             0x80U) == 0U)
  {
    return 0U;
  }
  BSP_DelayMs(100U);

  if (BSP_I2C2_WriteRegister(g_mpu6050_address,
                             MPU6050_REG_PWR_MGMT_1,
                             0x01U) == 0U)
  {
    return 0U;
  }
  BSP_DelayMs(50U);

  /* 采样率 100 Hz，低通滤波约 44 Hz，陀螺仪量程正负 2000 dps。 */
  if (BSP_I2C2_WriteRegister(g_mpu6050_address,
                             MPU6050_REG_SMPLRT_DIV,
                             0x09U) == 0U)
  {
    return 0U;
  }
  if (BSP_I2C2_WriteRegister(g_mpu6050_address,
                             MPU6050_REG_CONFIG,
                             0x03U) == 0U)
  {
    return 0U;
  }
  if (BSP_I2C2_WriteRegister(g_mpu6050_address,
                             MPU6050_REG_GYRO_CONFIG,
                             0x18U) == 0U)
  {
    return 0U;
  }
  if (BSP_I2C2_WriteRegister(g_mpu6050_address,
                             MPU6050_REG_ACCEL_CONFIG,
                             0x00U) == 0U)
  {
    return 0U;
  }

  return 1U;
}

uint8_t MPU6050_ReadGyroRaw(int16_t *gyro_x,
                            int16_t *gyro_y,
                            int16_t *gyro_z)
{
  if ((gyro_x == 0) || (gyro_y == 0) || (gyro_z == 0))
  {
    return 0U;
  }

  if (MPU6050_ReadInt16(MPU6050_REG_GYRO_XOUT_H, gyro_x) == 0U)
  {
    return 0U;
  }
  if (MPU6050_ReadInt16((uint8_t)(MPU6050_REG_GYRO_XOUT_H + 2U),
                        gyro_y) == 0U)
  {
    return 0U;
  }
  if (MPU6050_ReadInt16((uint8_t)(MPU6050_REG_GYRO_XOUT_H + 4U),
                        gyro_z) == 0U)
  {
    return 0U;
  }

  return 1U;
}

uint8_t MPU6050_CalibrateGyroZ(uint16_t samples,
                               uint16_t sample_delay_ms,
                               float *bias_raw)
{
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int32_t sum = 0;
  uint16_t i;

  if ((samples == 0U) || (bias_raw == 0))
  {
    return 0U;
  }

  for (i = 0U; i < samples; ++i)
  {
    if (MPU6050_ReadGyroRaw(&gyro_x, &gyro_y, &gyro_z) == 0U)
    {
      return 0U;
    }
    sum += (int32_t)gyro_z;
    BSP_DelayMs(sample_delay_ms);
  }

  *bias_raw = (float)sum / (float)samples;
  return 1U;
}
