#include "stm32f4xx.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "main.h"

void controlLoop(void)
{
		cmd.heart++;
		if(cmd.heart%1000==0)//��ʼ����������0.5����һ��
	{
		Monitor_Update();
	}
	#if TIM_DEBUG
		LED_HEAT();
	#endif
}





      