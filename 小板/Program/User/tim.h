#ifndef __TIM_H
#define __TIM_H
#include "stm32f4xx.h"


#define PWM1 TIM5->CCR1
#define PWM2 TIM5->CCR2
//Ä¦²ÁÂÖËÙ¶ÈÉèÖÃ
#define FRISpeed_Set(x)  \
	PWM1 = x;        \
	PWM2 = x;    
	
#define POK_E_ON \
GPIOA->BSRRL = GPIO_Pin_2; \
//TIM9->CCR1 = 20;

#define POK_E_OFF \
GPIOA->BSRRH =GPIO_Pin_2;\
//TIM9->CCR1 = 0;

void Bsp_Tim_Init(void);
uint32_t Get_Time_Micros(void);
void sysEnable(void);


#endif

