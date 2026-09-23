#include "tracking.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stddef.h>

void Tracking_ConfigPins(GPIO_TypeDef *port,
                         const uint16_t pins[TRACKING_SENSOR_COUNT])
{
  GPIO_InitTypeDef gpio;

  if ((port == NULL) || (pins == NULL))
  {
    return;
  }

  if (port == GPIOA)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  }
  else if (port == GPIOB)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  }
  else if (port == GPIOC)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  }

  gpio.GPIO_Pin = pins[0] | pins[1] | pins[2] | pins[3];
  /* 循迹模块输出是数字电平，使用上拉输入避免未接时悬空。 */
  gpio.GPIO_Mode = GPIO_Mode_IPU;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(port, &gpio);
}

void Tracking_Init(TrackingSensor_t *tracking,
                   GPIO_TypeDef *port,
                   const uint16_t pins[TRACKING_SENSOR_COUNT],
                   BitAction active_state,
                   uint8_t reverse_order)
{
  uint8_t i;

  if ((tracking == NULL) || (port == NULL) || (pins == NULL))
  {
    return;
  }

  tracking->port = port;
  for (i = 0U; i < TRACKING_SENSOR_COUNT; ++i)
  {
    tracking->pins[i] = pins[i];
  }
  tracking->active_state = active_state;
  tracking->reverse_order = (reverse_order != 0U) ? 1U : 0U;
  tracking->active_mask = 0U;
  tracking->line_found = 0U;
  tracking->error = 0.0f;
}

void Tracking_Update(TrackingSensor_t *tracking)
{
  static const float weights[TRACKING_SENSOR_COUNT] = {-3.0f, -1.0f, 1.0f, 3.0f};
  uint8_t active_mask = 0U;
  uint8_t sensor_mask;
  uint8_t i;
  uint8_t count = 0U;
  float sum = 0.0f;

  if (tracking == NULL)
  {
    return;
  }

  for (i = 0U; i < TRACKING_SENSOR_COUNT; ++i)
  {
    /* 读取有效电平，并把结果保存到 bit0 到 bit3。 */
    if (GPIO_ReadInputDataBit(tracking->port, tracking->pins[i]) == tracking->active_state)
    {
      active_mask |= (uint8_t)(1U << i);
    }
  }

  /* 如果传感器安装顺序与小车左右方向相反，就在软件里镜像四个 bit。 */
  if (tracking->reverse_order != 0U)
  {
    sensor_mask = (uint8_t)(((active_mask & 0x01U) << 3U) |
                            ((active_mask & 0x02U) << 1U) |
                            ((active_mask & 0x04U) >> 1U) |
                            ((active_mask & 0x08U) >> 3U));
  }
  else
  {
    sensor_mask = active_mask;
  }

  tracking->active_mask = sensor_mask;
  tracking->line_found = (sensor_mask != 0U) ? 1U : 0U;

  if (sensor_mask == 0U)
  {
    return;
  }

  for (i = 0U; i < TRACKING_SENSOR_COUNT; ++i)
  {
    if ((sensor_mask & (uint8_t)(1U << i)) != 0U)
    {
      /* 四个权重分别是 -3、-1、+1、+3。 */
      sum += weights[i];
      ++count;
    }
  }

  /* 多个传感器同时检测到线时，取平均位置作为本次误差。 */
  tracking->error = sum / (float)count;
}
