#ifndef __IST8310_H
#define __IST8310_H
#include "stm32f4xx.h"

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

/*---IST8310 cross-axis matrix Address-----------------danny-----*/
#define IST8310_REG_XX_CROSS_L       0x9C   //cross axis xx low byte
#define IST8310_REG_XX_CROSS_H       0x9D   //cross axis xx high byte
#define IST8310_REG_XY_CROSS_L       0x9E   //cross axis xy low byte
#define IST8310_REG_XY_CROSS_H       0x9F   //cross axis xy high byte
#define IST8310_REG_XZ_CROSS_L       0xA0   //cross axis xz low byte
#define IST8310_REG_XZ_CROSS_H       0xA1   //cross axis xz high byte

#define IST8310_REG_YX_CROSS_L       0xA2   //cross axis yx low byte
#define IST8310_REG_YX_CROSS_H       0xA3   //cross axis yx high byte
#define IST8310_REG_YY_CROSS_L       0xA4   //cross axis yy low byte
#define IST8310_REG_YY_CROSS_H       0xA5   //cross axis yy high byte
#define IST8310_REG_YZ_CROSS_L       0xA6   //cross axis yz low byte
#define IST8310_REG_YZ_CROSS_H       0xA7   //cross axis yz high byte

#define IST8310_REG_ZX_CROSS_L       0xA8   //cross axis zx low byte
#define IST8310_REG_ZX_CROSS_H       0xA9   //cross axis zx high byte
#define IST8310_REG_ZY_CROSS_L       0xAA   //cross axis zy low byte
#define IST8310_REG_ZY_CROSS_H       0xAB   //cross axis zy high byte
#define IST8310_REG_ZZ_CROSS_L       0xAC   //cross axis zz low byte
#define IST8310_REG_ZZ_CROSS_H       0xAD   //cross axis zz high byte


typedef struct 
{
	int16_t mx;
	int16_t my;
	int16_t mz;
}magDatadef;
void Ist8310_Init(void);
void IST8310_GetData(magDatadef *m);
void Ist8310_CrossaxisTransformation(float crossaxis_inv[9],magDatadef *m,magDatadef *o);
void Ist8310_DataUpdate(void);

void IST8310_SetStatus(uint8_t x);
uint8_t IST8310_GetStatus(void);
void IST8310_SetDataStatus(uint8_t x);
uint8_t IST8310_GetDataStatus(void);

uint8_t IST8310_GetIntStatus(uint8_t flag);
#endif
