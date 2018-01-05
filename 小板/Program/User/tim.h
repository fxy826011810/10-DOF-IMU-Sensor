#ifndef __TIM_H__
#define __TIM_H__
#include "stm32f4xx.h"

void Bsp_Tim_Init(void);
uint32_t Get_Time_Micros(void);
void sysEnable(void);


#endif

