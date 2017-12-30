#include "main.h"
void Bsp_NVIC_Init(void)
{
	NVIC_InitTypeDef											nvic;


	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


	//usart1
	nvic.NVIC_IRQChannel											= USART1_IRQn;
	nvic.NVIC_IRQChannelCmd										= ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority		= 0;
	nvic.NVIC_IRQChannelSubPriority						= 0;
	NVIC_Init(&nvic);

	//usart3
	nvic.NVIC_IRQChannel 											= USART3_IRQn;
	nvic.NVIC_IRQChannelCmd 									= ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority 		= 0;
	nvic.NVIC_IRQChannelSubPriority 					= 1;
	NVIC_Init(&nvic);


//	//can1
//	nvic.NVIC_IRQChannel											= CAN1_RX0_IRQn;
//	nvic.NVIC_IRQChannelCmd										= ENABLE;
//	nvic.NVIC_IRQChannelPreemptionPriority		= 1;
//	nvic.NVIC_IRQChannelSubPriority						= 1;
//	NVIC_Init(&nvic);
//	//can2
//	nvic.NVIC_IRQChannel											= CAN2_RX0_IRQn;
//	nvic.NVIC_IRQChannelCmd										= ENABLE;
//	nvic.NVIC_IRQChannelPreemptionPriority		= 1;
//	nvic.NVIC_IRQChannelSubPriority						= 2;
//	NVIC_Init(&nvic);



	//tim6
	nvic.NVIC_IRQChannel											= TIM6_DAC_IRQn;
	nvic.NVIC_IRQChannelCmd										= ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority		= 1;
	nvic.NVIC_IRQChannelSubPriority						= 3;
	NVIC_Init(&nvic);
	
		//ÍÓÂÝÒÇÖÐ¶Ï
  nvic.NVIC_IRQChannel = EXTI3_IRQn;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&nvic);
}
