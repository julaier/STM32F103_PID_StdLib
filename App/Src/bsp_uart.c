#include "bsp_uart.h"
#include "app_config.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include <stddef.h>

void BSP_UART_Init(void)
{
  GPIO_InitTypeDef gpio;
  USART_InitTypeDef usart;

  /* USART1 使用 PA9 作为 TX，PA10 作为 RX。 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

  /* PA9 配成复用推挽输出，作为 USART1_TX。 */
  gpio.GPIO_Pin = GPIO_Pin_9;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio);

  /* PA10 配成浮空输入，作为 USART1_RX。 */
  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio);

  /* 配置为 115200、8 数据位、1 停止位、无校验。 */
  usart.USART_BaudRate = DEBUG_UART_BAUD;
  usart.USART_WordLength = USART_WordLength_8b;
  usart.USART_StopBits = USART_StopBits_1;
  usart.USART_Parity = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &usart);
  USART_Cmd(USART1, ENABLE);
}

void BSP_UART_SendByte(uint8_t data)
{
  /* 等待发送寄存器为空，再把一个字节写入 USART1。 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
  {
  }

  USART_SendData(USART1, data);
}

void BSP_UART_SendBuffer(const uint8_t *data, uint16_t length)
{
  uint16_t i;

  if (data == NULL)
  {
    return;
  }

  for (i = 0U; i < length; ++i)
  {
    BSP_UART_SendByte(data[i]);
  }
}

void BSP_UART_SendString(const char *text)
{
  if (text == NULL)
  {
    return;
  }

  while (*text != '\0')
  {
    BSP_UART_SendByte((uint8_t)*text);
    ++text;
  }
}
