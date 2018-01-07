#include "stm32f4xx.h"
#include "dma.h"
#include "stdio.h"
extern u8 data_to_send[50];
void Bsp_DMA_Init(void)
{
	DMA_InitTypeDef dma;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2| RCC_AHB1Periph_DMA1, ENABLE);//´ò¿ªÊ±ÖÓ

	DMA_DeInit(DMA1_Stream3);
	DMA_StructInit(&dma);
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_BufferSize = 0;
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma.DMA_FIFOMode = DMA_FIFOMode_Enable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_Memory0BaseAddr = (uint32_t)NULL;;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Init(DMA1_Stream3, &dma);
	
	DMA_DeInit(DMA2_Stream7);
	DMA_StructInit(&dma);
	dma.DMA_Mode = DMA_Mode_Normal;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_BufferSize = 0;
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	dma.DMA_FIFOMode = DMA_FIFOMode_Enable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_Memory0BaseAddr =(uint32_t)NULL;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Init(DMA2_Stream7, &dma);
	
}



