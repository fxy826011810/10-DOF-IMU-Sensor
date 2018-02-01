#ifndef __ICM20602DRI_H__
#define __ICM20602DRI_H__
#include "stm32f4xx.h"


uint8_t Icm20602_WriteByte(uint8_t reg,uint8_t pbuffer);
uint8_t Icm20602_ReadByte(uint8_t reg,uint8_t *pbuffer);
uint8_t Icm20602_ReadBytes(uint8_t reg,uint8_t *pbuffer,uint8_t num);

#endif
