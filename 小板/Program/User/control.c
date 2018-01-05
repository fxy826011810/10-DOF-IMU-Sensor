#include "stm32f4xx.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "main.h"

void controlLoop(void)
{
		cmd.heart++;
		if(cmd.heart%1000==0)//初始化快闪正常0.5秒闪一次
	{
		Monitor_Update();
	}
	#if TIM_DEBUG
		LED_HEAT();
	#endif
}





      