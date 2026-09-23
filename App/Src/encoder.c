#include "encoder.h"
#include "app_config.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include <stddef.h>

static uint8_t Encoder_ConfigPins(TIM_TypeDef *timer)
{
  GPIO_InitTypeDef gpio;

  /* 编码器 A/B 相是数字输入，这里设置上拉输入以降低悬空干扰。 */
  gpio.GPIO_Mode = GPIO_Mode_IPU;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;

  /* 左编码器：PA0=TIM2_CH1(A相)，PA1=TIM2_CH2(B相)。 */
  if (timer == TIM2)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &gpio);
    return 1U;
  }

  /* 右编码器：PB6=TIM4_CH1(A相)，PB7=TIM4_CH2(B相)。 */
  if (timer == TIM4)
  {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &gpio);
    return 1U;
  }

  return 0U;
}

static void Encoder_ConfigTimer(TIM_TypeDef *timer)
{
  TIM_TimeBaseInitTypeDef time_base;
  TIM_ICInitTypeDef input_capture;

  /* 编码器模式不需要分频，使用 16 位计数器，满值会自动回绕。 */
  time_base.TIM_Prescaler = 0U;
  time_base.TIM_CounterMode = TIM_CounterMode_Up;
  time_base.TIM_Period = 0xFFFFU;
  time_base.TIM_ClockDivision = TIM_CKD_DIV1;
  time_base.TIM_RepetitionCounter = 0U;
  TIM_TimeBaseInit(timer, &time_base);

  /* 配置两个捕获输入通道，再由编码器接口自动判断旋转方向。 */
  input_capture.TIM_Channel = TIM_Channel_1;
  input_capture.TIM_ICPolarity = TIM_ICPolarity_Rising;
  input_capture.TIM_ICSelection = TIM_ICSelection_DirectTI;
  input_capture.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  input_capture.TIM_ICFilter = ENCODER_IC_FILTER;
  TIM_ICInit(timer, &input_capture);

  input_capture.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(timer, &input_capture);

  /* TI1 和 TI2 同时计数，即常说的四倍频正交编码器模式。 */
  TIM_EncoderInterfaceConfig(timer,
                             TIM_EncoderMode_TI12,
                             TIM_ICPolarity_Rising,
                             TIM_ICPolarity_Rising);

  TIM_SetCounter(timer, 0U);
  TIM_Cmd(timer, ENABLE);
}

uint8_t Encoder_Init(Encoder_t *encoder, TIM_TypeDef *timer, int8_t direction)
{
  if ((encoder == NULL) || (timer == NULL) || (direction == 0))
  {
    return 0U;
  }

  if ((timer != TIM2) && (timer != TIM4))
  {
    return 0U;
  }

  /* TIM2 和 TIM4 都挂在 APB1 总线上。 */
  RCC_APB1PeriphClockCmd((timer == TIM2) ? RCC_APB1Periph_TIM2 : RCC_APB1Periph_TIM4,
                         ENABLE);

  if (Encoder_ConfigPins(timer) == 0U)
  {
    return 0U;
  }

  Encoder_ConfigTimer(timer);

  encoder->timer = timer;
  encoder->last_raw_count = 0U;
  encoder->delta_count = 0;
  encoder->total_count = 0;
  encoder->speed_counts_s = 0.0f;
  encoder->direction = (direction < 0) ? -1 : 1;
  encoder->initialized = 1U;
  return 1U;
}

void Encoder_Reset(Encoder_t *encoder)
{
  if ((encoder == NULL) || (encoder->timer == NULL))
  {
    return;
  }

  TIM_SetCounter(encoder->timer, 0U);
  encoder->last_raw_count = 0U;
  encoder->delta_count = 0;
  encoder->total_count = 0;
  encoder->speed_counts_s = 0.0f;
}

int16_t Encoder_Update(Encoder_t *encoder, uint32_t dt_ms)
{
  uint16_t now;
  int32_t delta;

  if ((encoder == NULL) || (encoder->timer == NULL) || (dt_ms == 0U))
  {
    return 0;
  }

  now = TIM_GetCounter(encoder->timer);
  delta = (int32_t)now - (int32_t)encoder->last_raw_count;

  /* 处理 16 位计数器从 65535 到 0 或从 0 到 65535 的回绕。 */
  if (delta > 32767)
  {
    delta -= 65536;
  }
  else if (delta < -32768)
  {
    delta += 65536;
  }

  delta *= (int32_t)encoder->direction;
  encoder->last_raw_count = now;
  encoder->delta_count = (int16_t)delta;
  encoder->total_count += delta;
  /* counts / 采样时间 = counts/s。当前控制周期为 10 ms。 */
  encoder->speed_counts_s = ((float)delta * 1000.0f) / (float)dt_ms;

  return encoder->delta_count;
}

float Encoder_GetSpeed(const Encoder_t *encoder)
{
  return (encoder == NULL) ? 0.0f : encoder->speed_counts_s;
}

int32_t Encoder_GetTotal(const Encoder_t *encoder)
{
  return (encoder == NULL) ? 0 : encoder->total_count;
}
