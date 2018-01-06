#include "stm32f4xx.h"
#include "tim.h"
#include "control.h"
#include "config.h"
void Bsp_Tim_Init(void)
{
				TIM_TimeBaseInitTypeDef      tim;

				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6|RCC_APB1Periph_TIM2, ENABLE);
//1ms的控制计时
				TIM_TimeBaseStructInit(&tim);
				tim.TIM_ClockDivision = TIM_CKD_DIV1;
				tim.TIM_CounterMode = TIM_CounterMode_Up;
				tim.TIM_Period = 999;
				tim.TIM_Prescaler = 84-1;
				TIM_TimeBaseInit(TIM6,&tim);
//用于系统时间						
				tim.TIM_Period = 0xFFFFFFFF;
				tim.TIM_Prescaler = 84 - 1;	 //1M 的时钟  
				tim.TIM_ClockDivision = TIM_CKD_DIV1;
				tim.TIM_CounterMode = TIM_CounterMode_Up;
				TIM_ARRPreloadConfig(TIM2, ENABLE);
				TIM_TimeBaseInit(TIM2, &tim);
				TIM_Cmd(TIM2, ENABLE);			
}
void sysEnable(void)
{
				TIM_Cmd(TIM6, ENABLE);
				TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
				TIM_ClearFlag(TIM6, TIM_FLAG_Update);
}


void TIM6_DAC_IRQHandler(void)//控制任务
{
				if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
				{
					TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
					TIM_ClearFlag(TIM6, TIM_FLAG_Update);
					controlLoop();
				}
}

uint32_t Get_Time_Micros(void)//读取陀螺仪积分时间
{
				return TIM2->CNT;
}
