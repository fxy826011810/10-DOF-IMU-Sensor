#include "stm32f4xx.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "main.h"
#include "gpio.h"
void controlLoop(void)
{
		cmd.heart++;
	if(cmd.heart%10==0)//��ʼ����������0.5����һ��
	{
#if USE_MS5611
			Ms5611_ReadD(&cmd.Ms5611.Status,&cmd.Ms5611.Data);
			cmd.Ms5611.monitor.set(&cmd.Ms5611.monitor);
#endif
	}
		if(cmd.heart%1000==0)//��ʼ����������0.5����һ��
	{
		Monitor_Update();
	}
	#if USE_MS5611
	
#endif
	#if TIM_DEBUG
		LED_HEAT();
	#endif
}





      