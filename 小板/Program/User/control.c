#include "main.h"


uint32_t heart=0;//������
void controlLoop(void)
{
		heart++;
		if(heart%1000==0)//��ʼ����������0.5����һ��
	{
		Monitor_Update();

	}
	#if TIM_DEBUG
		LED_HEAT();
	#endif
}





      