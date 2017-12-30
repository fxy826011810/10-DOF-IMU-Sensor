#include "main.h"



uint8_t a;
void Bsp_Spi_Init(void)
{
	SPI_InitTypeDef spi;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPIX,ENABLE);
	SPI_I2S_DeInit(SPIX);
	spi.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8;
	spi.SPI_CPHA=SPI_CPHA_2Edge;
	spi.SPI_CPOL=SPI_CPOL_High;
	spi.SPI_CRCPolynomial=7;
	spi.SPI_DataSize=SPI_DataSize_8b;
	spi.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	spi.SPI_FirstBit=SPI_FirstBit_MSB;
	spi.SPI_Mode=SPI_Mode_Master;
	spi.SPI_NSS=SPI_NSS_Soft;
	SPI_Init(SPIX,&spi);
	SPI_Cmd(SPIX,ENABLE);
}


uint16_t SPI_WriteReadByte(uint8_t reg)
{	
	while(SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPIX, reg);
	while(SPI_I2S_GetFlagStatus(SPIX, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPIX);
}

