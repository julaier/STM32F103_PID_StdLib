#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f10x.h"
#include <stdint.h>

typedef struct
{
  /* 该编码器使用的硬件定时器，左轮是 TIM2，右轮是 TIM4。 */
  TIM_TypeDef *timer;
  /* 上一次的 16 位计数器原始值，用来计算本次采样增量。 */
  uint16_t last_raw_count;
  /* 本次采样增加的计数值，正数表示向前，负数表示向后。 */
  int16_t delta_count;
  /* 从复位开始累计的总计数值。 */
  int32_t total_count;
  /* 换算后的速度，单位是 counts/s。 */
  float speed_counts_s;
  /* 软件方向系数，1 或 -1，用于统一左右轮前进方向。 */
  int8_t direction;
  /* 1 表示编码器已经初始化并开始计数。 */
  uint8_t initialized;
} Encoder_t;

/* 初始化并启动一个 TIM2/TIM4 编码器接口。 */
uint8_t Encoder_Init(Encoder_t *encoder, TIM_TypeDef *timer, int8_t direction);
/* 清零计数器、累计值以及上次的原始计数。 */
void Encoder_Reset(Encoder_t *encoder);
/* 按 dt_ms 时间间隔更新一次速度，返回本次计数增量。 */
int16_t Encoder_Update(Encoder_t *encoder, uint32_t dt_ms);
float Encoder_GetSpeed(const Encoder_t *encoder);
int32_t Encoder_GetTotal(const Encoder_t *encoder);

#endif
