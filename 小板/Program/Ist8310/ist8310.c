#include "ist8310Dri.h"
#include "stm32f4xx.h"
#include "ist8310.h"
#include "config.h"
#include "delay.h"
//#include "gpio.h"
#include "main.h"
#include "i2c.h" 
uint8_t crossxbuf[6];
uint8_t crossybuf[6];
uint8_t crosszbuf[6];
void IST8310_GetCrossaxisData(void)
{
	IST8310_Read(IST8310_REG_XX_CROSS_L,crossxbuf,6);
	IST8310_Read(IST8310_REG_YX_CROSS_L,crossybuf,6);
	IST8310_Read(IST8310_REG_ZX_CROSS_L,crosszbuf,6);
}
void Ist8310_Init(void)
{
	ist8310IIC.writedataflag=1;
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
	IST8310_GetCrossaxisData();
	
}
uint8_t IST8310_GetIntData(uint8_t flag)
{
	uint8_t a;
	if(flag)
	return GPIO_ReadInputDataBit(IST8310_INT_GPIO,IST8310_INT_PIN);
	else
	{
	 IST8310_ReadByte(IST8310_R_MODE,&a);
	return a&0x01;
	}
	return 0;
}
void IST8310_GetMagData(uint8_t *data)
{
//	LED(0);
	IST8310_Read(IST8310_R_XL,data,6);
//	LED(1);
	IST8310_WriteByte(IST8310_R_CONFA,IST8310_ODR_MODE);
	
}
uint8_t IST8310_GetStatus(void)
{
	return cmd.Ist8310.status;
}
void IST8310_SetStatus(uint8_t x)
{
	cmd.Ist8310.status=x;
}
void IST8310_GetData(magDatadef *m)
{
	uint8_t data[6];
	IST8310_GetMagData(data);
	m->mx=data[1]<<8|data[0];
	m->my=data[3]<<8|data[2];
	m->mz=data[5]<<8|data[4];
}






