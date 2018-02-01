#include "stm32f4xx.h" 
#include "ms5611Dri.h" 
#include "config.h" 
#include "i2c.h" 

void Ms5611_WriteByte(uint8_t reg, uint8_t pbuffer)
{
#if USE_SIMIIC
	ms5611IIC.WriteByte(&ms5611IIC,reg,pbuffer);
#else
	IIC_H_WriteByte(I2CM,MS5611_ADDR,reg,pbuffer);
#endif	
}



void Ms5611_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len)
{
#if USE_SIMIIC
	ms5611IIC.ReadBytes(&ms5611IIC,reg,pbuffer,len);
#else
	IIC_H_Read(I2CM,MS5611_ADDR,reg,pbuffer,len);
#endif
}



void Ms5611_ReadByte(uint8_t reg, uint8_t *pbuffer)
{
#if USE_SIMIIC
	ms5611IIC.ReadByte(&ms5611IIC,reg,pbuffer);
#else
	IIC_H_Read(I2CM,MS5611_ADDR,reg,pbuffer,1);
#endif
}









