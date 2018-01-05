#include "stm32f4xx.h"
#include "icm20602Int.h"
#include "config.h"
void Icm20602IntInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  EXTI_InitTypeDef exti;
	
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);//ÍÓÂÝÒÇÖÐ¶Ï
  exti.EXTI_Line = EXTI_Line3;
  exti.EXTI_LineCmd = ENABLE;
  exti.EXTI_Mode = EXTI_Mode_Interrupt;
  exti.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&exti);
}



void EXTI3_IRQHandler(void)
{
	
	if (EXTI_GetITStatus(EXTI_Line3) != 0)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);
		EXTI_ClearFlag(EXTI_Line3);
	}
}
