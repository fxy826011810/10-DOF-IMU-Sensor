#ifndef __SPI_H
#define __SPI_H
#include "stm32f4xx.h"


void Bsp_Spi_Init(void);

uint16_t SPI_WriteReadByte(uint8_t reg);

#endif
