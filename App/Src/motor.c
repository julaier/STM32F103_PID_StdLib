#include "motor.h"
#include "app_config.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include <stddef.h>

static void Motor_ConfigTimer(TIM_TypeDef *timer)
{
  TIM_TimeBaseInitTypeDef time_base;

  /* TIM3 位于 APB1，时钟为 72 MHz。 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* 分频和周期：72 MHz / (71+1) / (999+1) = 1 kHz PWM。
     比较值范围 0 到 999，对应占空比 0% 到 100%。 */
  time_base.TIM_Prescaler = 71U;
  time_base.TIM_CounterMode = TIM_CounterMode_Up;
  time_base.TIM_Period = 999U;
  time_base.TIM_ClockDivision = TIM_CKD_DIV1;
  time_base.TIM_RepetitionCounter = 0U;
  TIM_TimeBaseInit(timer, &time_base);
  TIM_ARRPreloadConfig(timer, ENABLE);
}

uint8_t Motor_Init(Motor_t *motor, const Motor_Config_t *config)
{
  GPIO_InitTypeDef gpio;
  TIM_OCInitTypeDef output_compare;

  if ((motor == NULL) || (config == NULL) || (config->timer != TIM3))
  {
    return 0U;
  }

  if ((config->pwm_channel != 1U) && (config->pwm_channel != 2U))
  {
    return 0U;
  }

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* PA2/PA3 或 PA4/PA5：普通推挽输出，用于控制 L298N 方向。 */
  gpio.GPIO_Pin = config->in1_pin | config->in2_pin;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(config->direction_port, &gpio);

  /* PA6/PA7：复用推挽输出，对应 TIM3_CH1/CH2。 */
  gpio.GPIO_Pin = config->pwm_pin;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(config->pwm_port, &gpio);

  Motor_ConfigTimer(config->timer);

  /* PWM1 模式下，比较值越大，输出高电平时间越长。 */
  output_compare.TIM_OCMode = TIM_OCMode_PWM1;
  output_compare.TIM_OutputState = TIM_OutputState_Enable;
  output_compare.TIM_OutputNState = TIM_OutputNState_Disable;
  output_compare.TIM_Pulse = 0U;
  output_compare.TIM_OCPolarity = TIM_OCPolarity_High;
  output_compare.TIM_OCNPolarity = TIM_OCNPolarity_High;
  output_compare.TIM_OCIdleState = TIM_OCIdleState_Reset;
  output_compare.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

  if (config->pwm_channel == 1U)
  {
    /* 左电机使用 TIM3_CH1(PA6)。 */
    TIM_OC1Init(config->timer, &output_compare);
    TIM_OC1PreloadConfig(config->timer, TIM_OCPreload_Enable);
  }
  else
  {
    /* 右电机使用 TIM3_CH2(PA7)。 */
    TIM_OC2Init(config->timer, &output_compare);
    TIM_OC2PreloadConfig(config->timer, TIM_OCPreload_Enable);
  }

  TIM_Cmd(config->timer, ENABLE);

  motor->timer = config->timer;
  motor->pwm_channel = config->pwm_channel;
  motor->pwm_port = config->pwm_port;
  motor->pwm_pin = config->pwm_pin;
  motor->direction_port = config->direction_port;
  motor->in1_pin = config->in1_pin;
  motor->in2_pin = config->in2_pin;
  motor->min_duty = config->min_duty;
  motor->forward_invert = config->forward_invert;
  motor->command = 0;

  Motor_Stop(motor);
  return 1U;
}

void Motor_SetCommand(Motor_t *motor, int16_t command)
{
  uint16_t duty;
  uint8_t forward;

  if (motor == NULL)
  {
    return;
  }

  if (command > (int16_t)MOTOR_MAX_DUTY)
  {
    command = (int16_t)MOTOR_MAX_DUTY;
  }
  else if (command < -(int16_t)MOTOR_MAX_DUTY)
  {
    command = -(int16_t)MOTOR_MAX_DUTY;
  }

  motor->command = command;

  if (command == 0)
  {
    /* 命令为 0 时直接释放输出，避免电机继续转动。 */
    Motor_Stop(motor);
    return;
  }

  /* 命令的绝对值就是 PWM 比较值。 */
  duty = (command > 0) ? (uint16_t)command : (uint16_t)(-command);
  if (duty < motor->min_duty)
  {
    duty = motor->min_duty;
  }

  /* forward_invert 用于在软件层交换正反转方向。 */
  forward = (command > 0) ? 1U : 0U;
  if (motor->forward_invert != 0U)
  {
    forward = (forward == 0U) ? 1U : 0U;
  }

  /* L298N 方向：IN1=1、IN2=0 正转；IN1=0、IN2=1 反转。 */
  if (forward != 0U)
  {
    GPIO_SetBits(motor->direction_port, motor->in1_pin);
    GPIO_ResetBits(motor->direction_port, motor->in2_pin);
  }
  else
  {
    GPIO_ResetBits(motor->direction_port, motor->in1_pin);
    GPIO_SetBits(motor->direction_port, motor->in2_pin);
  }

  /* 将占空比写入对应通道的比较寄存器。 */
  if (motor->pwm_channel == 1U)
  {
    TIM_SetCompare1(motor->timer, duty);
  }
  else
  {
    TIM_SetCompare2(motor->timer, duty);
  }
}

void Motor_Stop(Motor_t *motor)
{
  if (motor == NULL)
  {
    return;
  }

  GPIO_ResetBits(motor->direction_port, motor->in1_pin);
  GPIO_ResetBits(motor->direction_port, motor->in2_pin);

  if (motor->pwm_channel == 1U)
  {
    TIM_SetCompare1(motor->timer, 0U);
  }
  else
  {
    TIM_SetCompare2(motor->timer, 0U);
  }

  motor->command = 0;
}

void Motor_Brake(Motor_t *motor)
{
  if (motor == NULL)
  {
    return;
  }

  GPIO_SetBits(motor->direction_port, motor->in1_pin);
  GPIO_SetBits(motor->direction_port, motor->in2_pin);

  if (motor->pwm_channel == 1U)
  {
    TIM_SetCompare1(motor->timer, 0U);
  }
  else
  {
    TIM_SetCompare2(motor->timer, 0U);
  }

  motor->command = 0;
}

int16_t Motor_GetCommand(const Motor_t *motor)
{
  return (motor == NULL) ? 0 : motor->command;
}
