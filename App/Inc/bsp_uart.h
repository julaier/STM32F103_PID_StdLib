#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

void BSP_UART_Init(void);
void BSP_UART_SendByte(uint8_t data);
void BSP_UART_SendBuffer(const uint8_t *data, uint16_t length);
void BSP_UART_SendString(const char *text);

#endif
