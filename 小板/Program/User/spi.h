#ifndef __SPI_H__
#define __SPI_H__
#include "stm32f4xx.h"


void Bsp_Spi_Init(void);

uint16_t SPI_WriteReadByte(uint8_t reg);

#endif
