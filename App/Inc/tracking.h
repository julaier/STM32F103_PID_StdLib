#ifndef TRACKING_H
#define TRACKING_H

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include <stdint.h>

#define TRACKING_SENSOR_COUNT 4U

typedef struct
{
  /* 四个传感器连接的 GPIO 端口和引脚。 */
  GPIO_TypeDef *port;
  uint16_t pins[TRACKING_SENSOR_COUNT];
  /* 黑色线对应的有效电平，常见模块为 Bit_RESET。 */
  BitAction active_state;
  /* 为 1 时把传感器 1 到 4 的左右顺序反过来。 */
  uint8_t reverse_order;
  /* bit0 到 bit3 对应从左到右四个传感器。 */
  uint8_t active_mask;
  /* 至少一个传感器检测到黑线时为 1。 */
  uint8_t line_found;
  /* 线位置误差：左偏为负，右偏为正，范围 -3 到 +3。 */
  float error;
} TrackingSensor_t;

void Tracking_ConfigPins(GPIO_TypeDef *port,
                         const uint16_t pins[TRACKING_SENSOR_COUNT]);
void Tracking_Init(TrackingSensor_t *tracking,
                   GPIO_TypeDef *port,
                   const uint16_t pins[TRACKING_SENSOR_COUNT],
                   BitAction active_state,
                   uint8_t reverse_order);
void Tracking_Update(TrackingSensor_t *tracking);

#endif
