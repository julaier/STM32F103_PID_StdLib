#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct
{
  /* PWM 定时器，本项目使用 TIM3。 */
  TIM_TypeDef *timer;
  /* PWM 通道：1 表示 TIM3_CH1，2 表示 TIM3_CH2。 */
  uint8_t pwm_channel;
  /* PWM 输出引脚及其端口，例如 PA6 或 PA7。 */
  GPIO_TypeDef *pwm_port;
  uint16_t pwm_pin;
  /* 方向控制 GPIO，例如 IN1/IN2 或 IN3/IN4。 */
  GPIO_TypeDef *direction_port;
  uint16_t in1_pin;
  uint16_t in2_pin;
  /* 非零命令的最小 PWM，用来补偿部分电机的静摩擦。 */
  uint16_t min_duty;
  /* 为 1 时交换软件正转和反转方向，不需要改接线。 */
  uint8_t forward_invert;
} Motor_Config_t;

typedef struct
{
  TIM_TypeDef *timer;
  uint8_t pwm_channel;
  GPIO_TypeDef *pwm_port;
  uint16_t pwm_pin;
  GPIO_TypeDef *direction_port;
  uint16_t in1_pin;
  uint16_t in2_pin;
  uint16_t min_duty;
  uint8_t forward_invert;
  /* 最近一次设置的命令，范围 -1000 到 +1000。 */
  int16_t command;
} Motor_t;

/* 初始化方向 GPIO、PWM GPIO 和 TIM3 PWM。 */
uint8_t Motor_Init(Motor_t *motor, const Motor_Config_t *config);
/* 设置电机命令：正数正转、负数反转、0 停止。 */
void Motor_SetCommand(Motor_t *motor, int16_t command);
/* 惯性停止：方向引脚都拉低，PWM 比较值清零。 */
void Motor_Stop(Motor_t *motor);
/* 主动刹车：IN1/IN2 同时拉高。 */
void Motor_Brake(Motor_t *motor);
int16_t Motor_GetCommand(const Motor_t *motor);

#endif
