#ifndef __IST8310_H
#define __IST8310_H
#include "stm32f4xx.h"
#define IST8310_ADDR          		0x0C
#define IST8310_WHO_AM_I          0x00
#define IST8310_R_CONFA           0x0A
#define IST8310_R_CONFB           0x0B
#define IST8310_R_MODE            0x02

#define IST8310_R_XL              0x03
#define IST8310_R_XM              0x04
#define IST8310_R_YL              0x05
#define IST8310_R_YM              0x06
#define IST8310_R_ZL              0x07
#define IST8310_R_ZM              0x08

#define IST8310_AVGCNTL           0x41
#define IST8310_PDCNTL            0x42

#define IST8310_ODR_MODE          0x01
typedef struct 
{
	int16_t mx;
	int16_t my;
	int16_t mz;
}magDatadef;

void Ist8310_Init(void);
void IST8310_ReadByte(uint8_t reg, uint8_t *pbuffer);
void IST8310_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len);
void IST8310_WriteByte(uint8_t reg, uint8_t pbuffer);
void IST8310_GetData(magDatadef *m);
uint8_t IST8310_GetIntData(void);
#endif