#include "stm32f4xx.h"
#include "icm20602.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "ms5611.h" 
#include "usart.h"
#include "ahrs.h"
#include "main.h"
void controlLoop(void)
{
		cmd.heart++;
	
	AHRS_Update();
	if(cmd.heart%10==0)
	{
#if USE_MS5611
		Ms5611_DataUpdate();
#endif
	}
		if(cmd.heart%1000==0)
	{
		Monitor_Update();
		LED_HEAT();
		if(cmd.Icm20602.monitor.count==0&&SPIInt==Icm20602_GetStatus())
		{
			Icm20602_SetStatus(Lost);
		}
		if(cmd.heart>=3000&&cmd.Icm20602.monitor.count==0)
		{
			Icm20602_SetStatus(SPIInt);
		}
	}
	if(cmd.heart%10==5)
	{
		if(AHRS_GetDataStatus())
		{
			Usart_Send_Angle(USART3,cmd.ahrs.angle[2],cmd.ahrs.angle[1],cmd.ahrs.angle[0]);
			Usart_Send_Status(USART1,cmd.ahrs.angle[2],cmd.ahrs.angle[1],cmd.ahrs.angle[0],0,0,0);
			AHRS_SetDataStatus(0);

		}
	}
	if(cmd.heart%10==0)
	{
//		Usart_Send_Senser(USART1,cmd.Icm20602.Data.ax,cmd.Icm20602.Data.ay,cmd.Icm20602.Data.az,cmd.Icm20602.Data.gx,cmd.Icm20602.Data.gy,cmd.Icm20602.Data.gz,cmd.Ist8310.Data.mx,cmd.Ist8310.Data.my,cmd.Ist8310.Data.mz,0);
		Usart_Send_Senser(USART1,cmd.Icm20602.Data.calc.ax,cmd.Icm20602.Data.calc.ay,cmd.Icm20602.Data.calc.az,
														 cmd.Ist8310.Data.original.mx,cmd.Ist8310.Data.original.my,cmd.Ist8310.Data.original.mz,
														 cmd.Ist8310.Data.calc.mx,cmd.Ist8310.Data.calc.my,cmd.Ist8310.Data.calc.mz,0);
	}
	
	#if USE_MS5611
	
#endif
	#if TIM_DEBUG
		LED_HEAT();
	#endif
}





      