#ifndef __IST8310DRI_H__
#define __IST8310DRI_H__
#include "stm32f4xx.h"

#define IST8310_ADDR          		0x0C

void IST8310_ReadByte(uint8_t reg, uint8_t *pbuffer);
void IST8310_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len);
void IST8310_WriteByte(uint8_t reg, uint8_t pbuffer);

#endif
