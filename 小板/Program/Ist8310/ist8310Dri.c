#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "ist8310Dri.h"
#include "config.h" 
#include "i2c.h" 


void IST8310_ReadByte(uint8_t reg, uint8_t *pbuffer)
{
	#if USE_SIMIIC
	ist8310IIC.ReadByte(&ist8310IIC,reg,pbuffer);
	#else
	IIC_H_Read(I2CI,IST8310_ADDR,reg,pbuffer,1);
	#endif
}

void IST8310_Read(uint8_t reg,uint8_t *pbuffer, uint8_t len)
{
	#if USE_SIMIIC
	ist8310IIC.ReadBytes(&ist8310IIC,reg,pbuffer,len);
	#else
	IIC_H_Read(I2CI,IST8310_ADDR,reg,pbuffer,len);
	#endif
	
}

void IST8310_WriteByte(uint8_t reg, uint8_t pbuffer)
{
	#if USE_SIMIIC
	ist8310IIC.WriteByte(&ist8310IIC,reg,pbuffer);
	#else
	IIC_H_WriteByte(I2CI,IST8310_ADDR,reg,pbuffer);
	#endif
}

