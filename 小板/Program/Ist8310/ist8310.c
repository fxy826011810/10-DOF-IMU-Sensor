#include "main.h"

void Ist8310_Init(void)
{
	ist8310IIC.Addr=IST8310_ADDR;
	ist8310IIC.ReadByte=&IIC_ReadByte;
	ist8310IIC.ReadBytes=&IIC_Read;
	ist8310IIC.WriteByte=&IIC_WriteByte;
	ist8310IIC.WriteBytes=&IIC_Write;
	
	uint8_t id1=0,c,d;
	IST8310_ReadByte(IST8310_WHO_AM_I,&id1);
	delay_ms(10);
	IST8310_WriteByte(IST8310_AVGCNTL, 0x24);
	delay_ms(10);
	IST8310_ReadByte(IST8310_AVGCNTL,&c);
	delay_ms(10);
	IST8310_WriteByte(IST8310_PDCNTL, 0xc0);
	delay_ms(10);
	IST8310_ReadByte(IST8310_PDCNTL,&d);
	delay_ms(10);
	IST8310_WriteByte(IST8310_R_CONFA,IST8310_ODR_MODE);
	delay_ms(10);
	
}
uint8_t IST8310_GetIntData(void)
{
	return GPIO_ReadInputDataBit(IST8310_INT_GPIO,IST8310_INT_PIN);
}
void IST8310_GetMagData(uint8_t *data)
{
	IST8310_Read(IST8310_R_XL,data,6);
	IST8310_WriteByte(IST8310_R_CONFA,IST8310_ODR_MODE);
}
magDatedef mag;
void IST8310_GetData(magDatedef *m)
{
	uint8_t data[6];
	IST8310_GetMagData(data);
	m->mx=data[1]<<8|data[0];
	m->my=data[3]<<8|data[2];
	m->mz=data[5]<<8|data[4];
}

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

