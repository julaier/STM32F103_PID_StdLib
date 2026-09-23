#ifndef BSP_I2C2_H
#define BSP_I2C2_H

#include <stdint.h>

/* 初始化 I2C2：SCL 使用 PB10，SDA 使用 PB11。 */
void BSP_I2C2_Init(void);

/* 向指定寄存器写一个字节。address7 是 7 位设备地址。 */
uint8_t BSP_I2C2_WriteRegister(uint8_t address7,
                               uint8_t reg,
                               uint8_t value);

/* 从指定寄存器读一个字节。address7 是 7 位设备地址。 */
uint8_t BSP_I2C2_ReadRegister(uint8_t address7,
                              uint8_t reg,
                              uint8_t *value);

#endif
