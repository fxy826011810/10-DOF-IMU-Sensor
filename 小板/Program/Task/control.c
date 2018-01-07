#include "stm32f4xx.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "main.h"
#include "gpio.h"
void controlLoop(void)
{
		cmd.heart++;
	
	if(IST8310_GetStatus()==1&&Icm20602_GetDataStatus()==1)
	{
		_9AxisAHRSupdate(&cmd.Icm20602.Data,&cmd.Ist8310.Data,&cmd.ahrs);
		Icm20602_SetDataStatus(0);
		IST8310_SetStatus(0);
	}
	else if(Icm20602_GetDataStatus()==1&&IST8310_GetStatus()==0)
	{
		_6AxisAHRSupdate(&cmd.Icm20602.Data,&cmd.ahrs);
		Icm20602_SetDataStatus(0);
	}
	
	if(cmd.heart%10==0)//初始化快闪正常0.5秒闪一次
	{
#if USE_MS5611
			Ms5611_ReadD(&cmd.Ms5611.Status,&cmd.Ms5611.Data);
			cmd.Ms5611.monitor.set(&cmd.Ms5611.monitor);
#endif
	}
		if(cmd.heart%1000==0)//初始化快闪正常0.5秒闪一次
	{
		Monitor_Update();
	}
	if(cmd.Icm20602.monitor.count==0&&cmd.Icm20602.Status==SPIInt)
	{
		cmd.Icm20602.Status=Lost;
	}
	if(cmd.heart>3000&&cmd.Icm20602.monitor.count==0)
	{
		cmd.Icm20602.Status=SPIInt;
	}
	
	#if USE_MS5611
	
#endif
	#if TIM_DEBUG
		LED_HEAT();
	#endif
}





      