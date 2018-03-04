#include "stm32f4xx.h"
#include "icm20602.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "ms5611.h" 
#include "usart.h"
#include "ahrs.h"
#include "main.h"
#include "tim.h"
#include "can.h"
#include <stdio.h>

void controlLoop(void)
{
		cmd.heart++;
	
	getFunctionTime(&AHRS_Update_t);
	if(cmd.heart%5==0)
	{
#if USE_MS5611
		getFunctionTime(&Ms5611_DataUpdate_t);
		Monitor_Set(&cmd.Ms5611->monitor);
#endif
	}
		if(cmd.heart%1000==0)
	{
		Monitor_Update();
		LED_HEAT();
		if(cmd.Icm20602->monitor.count==0&&SPIInt==Icm20602_GetStatus())
		{
			Icm20602_SetStatus(Lost);
		}
		if(cmd.heart>=3000&&cmd.Icm20602->monitor.count==0)
		{
			Icm20602_SetStatus(SPIInt);
		}
	}
	if(cmd.heart%10==0)
	{
		if(AHRS_GetDataStatus())
		{
			Can_AngleSend(CAN2,cmd.Ahrs->angle);
			Usart_Send_Angle(USART3,cmd.Ahrs->angle);
			Usart_Send_Status(USART1,cmd.Ahrs->angle,0,0,0);
			AHRS_SetDataStatus(0);
		}
	}
	
	
	if(cmd.heart%10==5)
	{
		Usart_Send_Senser(USART1,&cmd.Icm20602->Data.original,&cmd.Ist8310->Data.original,0);
	}
	
	#if USE_MS5611
	
#endif
	#if TIM_DEBUG
		LED_HEAT();
	#endif
	
	
}






