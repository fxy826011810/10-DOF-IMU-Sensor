#include "ist8310Int.h" 
#include "stm32f4xx.h"

#include "ist8310.h"
#include "config.h"
#include "main.h"
#include "imu.h"
void Ist8310IntInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  EXTI_InitTypeDef exti;
	
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);//ÍÓÂÝÒÇÖÐ¶Ï
  exti.EXTI_Line = EXTI_Line0;
  exti.EXTI_LineCmd = ENABLE;
  exti.EXTI_Mode = EXTI_Mode_Interrupt;
  exti.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&exti);
}



void Ist8310Int_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0)!= 0)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		EXTI_ClearFlag(EXTI_Line0);
	}
}

