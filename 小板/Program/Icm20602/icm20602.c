#include "stm32f4xx.h"
#include "icm20602.h"
#include "icm20602Dri.h"
#include "config.h"
#include "delay.h"
#include "main.h"
uint8_t Icm20602_init(void)
{
	cmd.Icm20602.Status=PinInt;
	cmd.Icm20602.DataStatus=0;
	uint8_t len=12;
	uint8_t i,initdata[][2]={ {ICM20602_PWR_MGMT_1,0x80},\
															{ICM20602_PWR_MGMT_1,0x01},\
															{ICM20602_INT_PIN_CFG,0x00},\
															{ICM20602_INT_ENABLE,0x01},\
															{ICM20602_PWR_MGMT_2,0x00},\
															{ICM20602_SMPLRT_DIV,0x00},\
															{ICM20602_GYRO_CONFIG,0x18},\
															{ICM20602_ACCEL_CONFIG,0x08},\
															{ICM20602_ACCEL_CONFIG_2,0x03},\
															{ICM20602_USER_CTRL,0x01},\
															{ICM20602_CONFIG,0x03},\
															{ICM20602_I2C_IF,0x40},\
														};
	uint8_t mpu6500_id=0,mpudata=0;	
	Icm20602_ReadByte(ICM20602_WHO_AM_I,&mpu6500_id);	
														
	if (mpu6500_id != ICM20602_WHO_AM_I_CONST)
	return 1; //–£—È ß∞‹£¨∑µªÿ0xff
	for(i=0;i<len;i++)
{
	Icm20602_WriteByte(initdata[i][0],initdata[i][1]);//Õ”¬›“«÷ÿ∆Ù
	delay_ms(100);
}
	return 0;
}
void Icm20602_SetDataStatus(uint8_t x)
{
	if(x)
	cmd.Icm20602.DataStatus=1;
	else
	cmd.Icm20602.DataStatus=0;
}
uint8_t Icm20602_GetDataStatus(void)
{
	return cmd.Icm20602.DataStatus;
}
uint8_t Icm20602_Calc(void)
{
	Icm20602Datadef offset;
	uint8_t i;int16_t temp[6]={0};
	for(i=0;i<calc_time;i++)
	{
		while(Icm20602_GetIntData())
		{
		Icm20602_GetData(&offset);
//		temp[0]=offset.ax;
//		temp[1]=offset.ay;
//		temp[2]=offset.az;
		temp[3]-=offset.gx;
		temp[4]-=offset.gy;
		temp[5]-=offset.gz;
		}
		delay_ms(10);
	}
	temp[3]/=calc_time/2;
	temp[4]/=calc_time/2;
	temp[5]/=calc_time/2;
	if(temp[3]<100&&temp[4]<100&&temp[5]<100)
	{
		Icm20602_WriteByte(ICM20602_XG_OFFS_USRH,(uint8_t)(temp[3]>>8));
		Icm20602_WriteByte(ICM20602_XG_OFFS_USRL,(uint8_t)(temp[3]&0xFF));
		Icm20602_WriteByte(ICM20602_YG_OFFS_USRH,(uint8_t)(temp[4]>>8));
		Icm20602_WriteByte(ICM20602_YG_OFFS_USRL,(uint8_t)(temp[4]&0xFF));
		Icm20602_WriteByte(ICM20602_ZG_OFFS_USRH,(uint8_t)(temp[5]>>8));
		Icm20602_WriteByte(ICM20602_ZG_OFFS_USRL,(uint8_t)(temp[5]&0xFF));
	}

}

#define Filter_time 5
void IMU_Filter(Icm20602Datadef *data,Icm20602Datadef *out)
{
	out->ax=((Filter_time-1)*out->ax+data->ax)/Filter_time;
	out->ay=((Filter_time-1)*out->ay+data->ay)/Filter_time;
	out->az=((Filter_time-1)*out->az+data->az)/Filter_time;
	out->gx=((Filter_time-1)*out->gx+data->gx)/Filter_time;
	out->gy=((Filter_time-1)*out->gy+data->gy)/Filter_time;
	out->gz=((Filter_time-1)*out->gz+data->gz)/Filter_time;
	out->temp=((Filter_time-1)*out->temp+data->temp)/Filter_time;
}

uint8_t Icm20602_DataLimit(Icm20602Datadef *data)
{
	if(data->gx<ICM20602LIMIT_MIN&&data->gx>(-ICM20602LIMIT_MIN))
		data->gx=0;
	if(data->gy<ICM20602LIMIT_MIN&&data->gy>(-ICM20602LIMIT_MIN))
		data->gy=0;
	if(data->gz<ICM20602LIMIT_MIN&&data->gz>(-ICM20602LIMIT_MIN))
		data->gz=0;
	return 0;
}

uint8_t Icm20602_GetIntData(void)
{
	uint8_t a;
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