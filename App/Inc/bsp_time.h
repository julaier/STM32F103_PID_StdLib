#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

void BSP_DelayMs(uint32_t ms);
void BSP_TimeInit(void);
uint16_t BSP_GetUs16(void);

#endif
