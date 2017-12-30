#include "main.h"


uint32_t heart=0;//心跳灯
void controlLoop(void)
{
		heart++;
		if(heart%1000==0)//初始化快闪正常0.5秒闪一次
	{
		Monitor_Update();
		LED_HEAT();
	}
	
}





      