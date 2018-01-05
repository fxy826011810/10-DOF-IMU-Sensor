#include "stm32f4xx.h"
#include "icm20602.h"
#include "icm20602Dri.h"
#include "config.h"
#include "delay.h"

uint8_t Icm20602_init(void)
{
	uint8_t len=11;
	uint8_t i,initdata[][2]={ {ICM20602_PWR_MGMT_1,0x80},\
															{ICM20602_PWR_MGMT_1,0x01},\
															{ICM20602_USER_CTRL,0x01},\
															{ICM20602_SMPLRT_DIV,0x00},\
															{ICM20602_GYRO_CONFIG,0x18},
															{ICM20602_ACCEL_CONFIG,0x08},\
															{ICM20602_ACCEL_CONFIG_2,0x03},\
															{ICM20602_CONFIG,0x03},\
															{ICM20602_INT_PIN_CFG,0x00},\
															{ICM20602_PWR_MGMT_2,0x00},\
															{ICM20602_I2C_IF,0x40},\
														};
	uint8_t mpu6500_id=0,mpudata[11];	
	Icm20602_ReadByte(ICM20602_WHO_AM_I,&mpu6500_id);	
														
	if (mpu6500_id != ICM20602_WHO_AM_I_CONST)
	return 1; //–£—È ß∞‹£¨∑µªÿ0xff
	for(i=0;i<len;i++)
{
	Icm20602_WriteByte(initdata[i][0],initdata[i][1]);//Õ”¬›“«÷ÿ∆Ù
	delay_ms(100);
}
	for(i=1;i<len;i++)
{
	Icm20602_ReadByte(initdata[i][0],&mpudata[i]);	
	delay_ms(1);
}
	return 0;
}

uint8_t Icm20602_GetIntData(void)
{
	uint8_t a;
//	return GPIO_ReadInputDataBit(SPIX_IRQ_GPIO,SPIX_IRQ_PIN);
	Icm20602_ReadByte(ICM20602_INT_STATUS,&a);
	return a&0x01;
}

void Icm20602_GetRegisterData(uint8_t *data)
{
	Icm20602_ReadBytes(ICM20602_ACCEL_XOUT_H,data,14);
}


void Icm20602_GetData(Icm20602Datadef *icmdata)
{
	uint8_t data[14];
	Icm20602_GetRegisterData(data);
	icmdata->ax=data[0]<<8|data[1];
	icmdata->ay=data[2]<<8|data[3];
	icmdata->az=data[4]<<8|data[5];
	icmdata->temp=data[6]<<8|data[7];
	icmdata->gx=data[8]<<8|data[9];
	icmdata->gy=data[10]<<8|data[11];
	icmdata->gz=data[12]<<8|data[13];
	
}