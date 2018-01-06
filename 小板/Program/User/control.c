#include "stm32f4xx.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "main.h"
#include "gpio.h"
void controlLoop(void)
{
		cmd.heart_t.heart++;
	if(cmd.heart_t.heart%10==0)//初始化快闪正常0.5秒闪一次
	{
			Ms5611_ReadD(&cmd.Ms5611.Status,&cmd.Ms5611.Data);
			cmd.Ms5611.monitor.time++;
	}
		if(cmd.heart_t.heart%1000==0)//初始化快闪正常0.5秒闪一次
	{
		Monitor_Update();
		LED_HEAT();
	}
	#if USE_MS5611
	
#endif
	#if TIM_DEBUG
		LED_HEAT();
	#endif
}





      