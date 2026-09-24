#include "bsp_time.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

void BSP_DelayMs(uint32_t ms)
{
  if (ms == 0U)
  {
    return;
  }

  /* 直接使用 SysTick 做一个阻塞式毫秒延时，不占用任何定时器。 */
  SysTick->CTRL = 0U;
  SysTick->LOAD = (SystemCoreClock / 1000U) - 1U;
  SysTick->VAL = 0U;

  while (ms != 0U)
  {
    /* 清零当前值，等待 COUNTFLAG 置位表示 1 ms 到。 */
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U)
    {
    }

    --ms;
  }

  SysTick->CTRL = 0U;
}

void BSP_TimeInit(void)
{
  TIM_TimeBaseInitTypeDef time_base;

  /* TIM1 计数频率 1 MHz，16 位计数器约 65.5 ms 回绕一次。 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  time_base.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000U) - 1U);
  time_base.TIM_CounterMode = TIM_CounterMode_Up;
  time_base.TIM_Period = 0xFFFFU;
  time_base.TIM_ClockDivision = TIM_CKD_DIV1;
  time_base.TIM_RepetitionCounter = 0U;
  TIM_TimeBaseInit(TIM1, &time_base);
  TIM_SetCounter(TIM1, 0U);
  TIM_Cmd(TIM1, ENABLE);
}

uint16_t BSP_GetUs16(void)
{
  return (uint16_t)TIM_GetCounter(TIM1);
}
