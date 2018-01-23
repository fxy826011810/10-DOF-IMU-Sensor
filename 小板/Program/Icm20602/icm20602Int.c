#include "stm32f4xx.h"
#include "icm20602Int.h"
#include "icm20602.h"
#include "main.h"
#include "imu.h"
void Icm20602IntInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  EXTI_InitTypeDef exti;
	
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);//�������ж�
  exti.EXTI_Line = EXTI_Line3;
  exti.EXTI_LineCmd = ENABLE;
  exti.EXTI_Mode = EXTI_Mode_Interrupt;
  exti.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&exti);
}



void Icm20602Int_IRQHandler(void)
{
	
	if (EXTI_GetITStatus(EXTI_Line3)!= 0)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);
		EXTI_ClearFlag(EXTI_Line3);
		
#if USE_ICM20602
		cmd.Icm20602.status=PinInt;
		if(Icm20602_GetStatus()==PinInt)
		{
			ICM20602_DataUpdate();
		}	
#endif
	}
}
