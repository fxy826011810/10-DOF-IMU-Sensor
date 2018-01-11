#include "stm32f4xx.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "main.h"
#include "gpio.h"
#include "usart.h"
void controlLoop(void)
{
		cmd.heart++;
	if(IST8310_GetStatus()==1&&Icm20602_GetDataStatus()==1)
	{
		_9AxisAHRSupdate(&cmd.Icm20602.Data,&cmd.Ist8310.Data,&cmd.ahrs);
		Icm20602_SetDataStatus(0);
		IST8310_SetStatus(0);
		cmd.ahrs.DataStatus=1;
	}
	else if(Icm20602_GetDataStatus()==1&&IST8310_GetStatus()==0)
	{
		_6AxisAHRSupdate(&cmd.Icm20602.Data,&cmd.ahrs);
		Icm20602_SetDataStatus(0);
		cmd.ahrs.DataStatus=1;
	}
	
	if(cmd.heart%10==0)
	{
#if USE_MS5611
			Ms5611_ReadD(&cmd.Ms5611.Status,&cmd.Ms5611.Data);
			cmd.Ms5611.monitor.set(&cmd.Ms5611.monitor);
#endif
	}
		if(cmd.heart%1000==0)
	{
		Monitor_Update();
		LED_HEAT();
		if(cmd.Icm20602.monitor.count==0&&cmd.Icm20602.Status==SPIInt)
		{
			cmd.Icm20602.Status=Lost;
		}
		if(cmd.heart>=3000&&cmd.Icm20602.monitor.count==0)
		{
			cmd.Icm20602.Status=SPIInt;
		}
	}
	if(cmd.heart%10==5)
	{
		if(cmd.ahrs.DataStatus)
		{
			cmd.ahrs.DataStatus=0;
			Usart_Send_Angle(USART3,cmd.ahrs.angle[2],cmd.ahrs.angle[1],cmd.ahrs.angle[0]);
			Usart_Send_Status(USART1,cmd.ahrs.angle[2],cmd.ahrs.angle[1],cmd.ahrs.angle[0],0,0,0);
			cmd.ahrs.DataStatus=0;
		}
	}
	if(cmd.heart%10==0)
	{
		Usart_Send_Senser(USART1,cmd.Icm20602.Data.ax,cmd.Icm20602.Data.ay,cmd.Icm20602.Data.az,cmd.Icm20602.Data.gx,cmd.Icm20602.Data.gy,cmd.Icm20602.Data.gz,cmd.Ist8310.Data.mx,cmd.Ist8310.Data.my,cmd.Ist8310.Data.mz,0);
	}
	
	#if USE_MS5611
	
#endif
	#if TIM_DEBUG
		LED_HEAT();
	#endif
}





      