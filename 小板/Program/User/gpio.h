#ifndef __GPIO_H
#define __GPIO_H
#include "stm32f4xx.h"
#define	LED_HEAT() GPIOC->ODR^=GPIO_Pin_12
#define LED(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_12):GPIO_ResetBits(GPIOC,GPIO_Pin_12)
void Bsp_GPIO_Init(void);

#endif
