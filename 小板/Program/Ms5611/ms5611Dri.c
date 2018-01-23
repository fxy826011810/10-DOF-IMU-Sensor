#include "i2c.h" 
#include "stm32f4xx.h" 
uint8_t Ms5611_WriteByte(uint8_t reg, uint8_t pbuffer)
{
	return ms5611IIC.WriteByte(&ms5611IIC,reg,pbuffer);
}



uint8_t Ms5611_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len)
{
	return ms5611IIC.ReadBytes(&ms5611IIC,reg,pbuffer,len);
}



uint8_t Ms5611_ReadByte(uint8_t reg, uint8_t *pbuffer)
{
	return ms5611IIC.ReadByte(&ms5611IIC,reg,pbuffer);
}









