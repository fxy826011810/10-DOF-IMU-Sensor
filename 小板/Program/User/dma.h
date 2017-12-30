#ifndef __DMA_H
#define __DMA_H
#include "stm32f4xx.h"
#define ImuBufferLength 20
extern uint8_t buffer[27];
void Bsp_DMA_Init(void);

#endif
