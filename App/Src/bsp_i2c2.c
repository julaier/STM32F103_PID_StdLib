#include "bsp_i2c2.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"

/* 通信失败时不能把主循环永久卡死，所以每次等待都有超时计数。 */
#define BSP_I2C2_TIMEOUT_COUNT 100000U

static uint8_t BSP_I2C2_WaitEvent(uint32_t event)
{
  uint32_t timeout = BSP_I2C2_TIMEOUT_COUNT;

  while (I2C_CheckEvent(I2C2, event) != SUCCESS)
  {
    if (timeout == 0U)
    {
      return 0U;
    }
    --timeout;
  }

  return 1U;
}

static uint8_t BSP_I2C2_WaitBusFree(void)
{
  uint32_t timeout = BSP_I2C2_TIMEOUT_COUNT;

  while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) != RESET)
  {
    if (timeout == 0U)
    {
      return 0U;
    }
    --timeout;
  }

  return 1U;
}

static void BSP_I2C2_ClearErrorFlags(void)
{
  /* 从机不应答时会置 AF，必须清除，否则下一次探测会继续失败。 */
  if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF) != RESET)
  {
    I2C_ClearFlag(I2C2, I2C_FLAG_AF);
  }
  if (I2C_GetFlagStatus(I2C2, I2C_FLAG_BERR) != RESET)
  {
    I2C_ClearFlag(I2C2, I2C_FLAG_BERR);
  }
  if (I2C_GetFlagStatus(I2C2, I2C_FLAG_ARLO) != RESET)
  {
    I2C_ClearFlag(I2C2, I2C_FLAG_ARLO);
  }
}

void BSP_I2C2_Init(void)
{
  GPIO_InitTypeDef gpio;
  I2C_InitTypeDef i2c;

  /* STM32F103 的 I2C2 默认使用 PB10 作为 SCL，PB11 作为 SDA。 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

  gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &gpio);

  I2C_DeInit(I2C2);

  i2c.I2C_Mode = I2C_Mode_I2C;
  i2c.I2C_DutyCycle = I2C_DutyCycle_2;
  i2c.I2C_OwnAddress1 = 0x00U;
  i2c.I2C_Ack = I2C_Ack_Enable;
  i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  i2c.I2C_ClockSpeed = 100000U;
  I2C_Init(I2C2, &i2c);

  I2C_Cmd(I2C2, ENABLE);
  I2C_AcknowledgeConfig(I2C2, ENABLE);
}

uint8_t BSP_I2C2_WriteRegister(uint8_t address7,
                               uint8_t reg,
                               uint8_t value)
{
  uint8_t success = 0U;

  if (BSP_I2C2_WaitBusFree() == 0U)
  {
    return 0U;
  }
  BSP_I2C2_ClearErrorFlags();

  I2C_GenerateSTART(I2C2, ENABLE);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == 0U)
  {
    goto exit;
  }

  I2C_Send7bitAddress(I2C2,
                      (uint8_t)(address7 << 1U),
                      I2C_Direction_Transmitter);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0U)
  {
    goto exit;
  }

  I2C_SendData(I2C2, reg);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0U)
  {
    goto exit;
  }

  I2C_SendData(I2C2, value);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0U)
  {
    goto exit;
  }

  success = 1U;

exit:
  I2C_AcknowledgeConfig(I2C2, ENABLE);
  if (success == 0U)
  {
    BSP_I2C2_ClearErrorFlags();
  }
  I2C_GenerateSTOP(I2C2, ENABLE);
  return success;
}

uint8_t BSP_I2C2_ReadRegister(uint8_t address7,
                              uint8_t reg,
                              uint8_t *value)
{
  uint8_t success = 0U;

  if ((value == 0) || (BSP_I2C2_WaitBusFree() == 0U))
  {
    return 0U;
  }
  BSP_I2C2_ClearErrorFlags();

  I2C_GenerateSTART(I2C2, ENABLE);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == 0U)
  {
    goto exit;
  }

  I2C_Send7bitAddress(I2C2,
                      (uint8_t)(address7 << 1U),
                      I2C_Direction_Transmitter);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0U)
  {
    goto exit;
  }

  I2C_SendData(I2C2, reg);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0U)
  {
    goto exit;
  }

  I2C_GenerateSTART(I2C2, ENABLE);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == 0U)
  {
    goto exit;
  }

  I2C_Send7bitAddress(I2C2,
                      (uint8_t)(address7 << 1U),
                      I2C_Direction_Receiver);
  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == 0U)
  {
    goto exit;
  }

  /* 单字节读取时，先发送 NACK 和 STOP，再读取数据寄存器。 */
  I2C_AcknowledgeConfig(I2C2, DISABLE);
  I2C_GenerateSTOP(I2C2, ENABLE);

  if (BSP_I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) == 0U)
  {
    goto exit;
  }

  *value = I2C_ReceiveData(I2C2);
  success = 1U;

exit:
  I2C_AcknowledgeConfig(I2C2, ENABLE);
  if (success == 0U)
  {
    BSP_I2C2_ClearErrorFlags();
    I2C_GenerateSTOP(I2C2, ENABLE);
  }
  return success;
}
