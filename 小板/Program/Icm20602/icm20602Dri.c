#include "stm32f4xx.h"
#include "icm20602Dri.h"
#include "config.h"
#include "delay.h"
#include "spi.h"
uint8_t Icm20602_WriteByte(uint8_t reg,uint8_t pbuffer)
{
	uint16_t data;
	reg=reg&0x7F;
	icm20602(0);
	SPI_WriteReadByte(reg);
	data=SPI_WriteReadByte(pbuffer);
	icm20602(1);
//	delay_us(5);
	return data;
}

uint8_t Icm20602_ReadByte(uint8_t reg,uint8_t *pbuffer)
{
	 
	icm20602(0);
	 SPI_WriteReadByte(reg|0x80);
	*pbuffer =SPI_WriteReadByte(0xff);
	icm20602(1);
//	delay_us(5);
	return 0;
}

uint8_t Icm20602_ReadBytes(uint8_t reg,uint8_t *pbuffer,uint8_t num)
{
	uint8_t i;
	icm20602(0);
	SPI_WriteReadByte(reg|0x80);
	for(i=0;i<num;i++)
	{	
	*pbuffer =SPI_WriteReadByte(0x00);
	pbuffer++;
	}
	icm20602(1);
//	delay_us(1);
	return 0;
}