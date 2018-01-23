#include "stm32f4xx.h"
#include "ist8310Dri.h"
#include "i2c.h" 


void IST8310_ReadByte(uint8_t reg, uint8_t *pbuffer)
{
	ist8310IIC.ReadByte(&ist8310IIC,reg,pbuffer);
}

void IST8310_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len)
{
	ist8310IIC.ReadBytes(&ist8310IIC,reg,pbuffer,len);
}

void IST8310_WriteByte(uint8_t reg, uint8_t pbuffer)
{
	ist8310IIC.WriteByte(&ist8310IIC,reg,pbuffer);
}

