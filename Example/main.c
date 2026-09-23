/* ============================================================================
 * main.c —— 整车程序入口
 *
 * 整个工程只有这一个 main 函数，它只做两件事：
 *   1. 上电后调用一次 Robot_UserSetup()，把硬件和 PID 初始化好；
 *   2. 然后进入死循环，反复调用 Robot_UserLoop()，不停地跑测试或控制任务。
 *
 * 注意：
 *   - 具体的测试内容和 7 种测试模式都在 Example/user_code.c 里，
 *     要换测试模式、改 PID 参数，都去那个文件改，不用动这个文件。
 *   - 本工程用 STM32F10x 标准外设库（StdPeriph），不是 HAL 库。
 * ========================================================================== */

#include "stm32f10x.h"
#include "user_code.h"

int main(void)
{
  /* 启动文件 startup_stm32f10x_md.s 在跳到 main 之前已经调用过 SystemInit()，
     系统时钟这时已经是 72 MHz 了，所以这里不需要再写时钟配置代码。 */

  /* 一次性初始化：
       USART1 调试串口 -> 左编码器 TIM2 -> 右编码器 TIM4
       -> 左电机 TIM3_CH1 -> 右电机 TIM3_CH2
       -> 循迹 PB12~PB15 -> 三个 PID 的参数和限幅。
     如果任何一个模块初始化失败，这个函数会在串口打印
     "Robot init failed." 然后停在那里，方便定位问题。 */
  Robot_UserSetup();

  /* 主循环：每次调用执行一个控制周期，
     周期长度由 ROBOT_CONTROL_PERIOD_MS 决定，当前是 10 ms。 */
  while (1)
  {
    Robot_UserLoop();
  }
}
