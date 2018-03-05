#include "stm32f4xx.h"
#include "icm20602Int.h"
#include "icm20602.h"
#include "config.h"
#include "main.h"
#include "imu.h"
#include "common.h"
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


uint32_t ac,bd;float ce;
void Icm20602Int_IRQHandler(void)
{
	
	if (EXTI_GetITStatus(EXTI_Line3)!= 0)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);
		EXTI_ClearFlag(EXTI_Line3);
		
#if USE_ICM20602
		__disable_irq();
		Icm20602_SetStatus(PinInt);
		if(Icm20602_GetStatus()==PinInt)
		{
			getFunctionTime(&ICM20602_DataUpdate_t);
			Monitor_Set(&cmd.Icm20602->monitor);
		}	
		__enable_irq();
#endif
	}
}
