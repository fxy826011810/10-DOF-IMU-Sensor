#ifndef __GPIO_H
#define __GPIO_H

#define	LED_HEAT() GPIOC->ODR^=GPIO_Pin_12

void Bsp_GPIO_Init(void);

#endif
