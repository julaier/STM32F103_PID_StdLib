#include "bsp_time.h"
#include "stm32f10x.h"

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
