#ifndef __USART_H
#define __USART_H
#include "stm32f4xx.h"
void Bsp_Usart_Init(void);
void Usart_DMASend(USART_TypeDef* USARTx,uint8_t *send,uint8_t len);
#endif

