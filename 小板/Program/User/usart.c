#include "stm32f4xx.h"
#include <stdio.h>
#include "usart.h"
int fputc(int ch,FILE*f)//printf函数重定义
{
	USART_SendData(USART3,(unsigned char)ch);
	while(!USART_GetFlagStatus(USART3,USART_FLAG_TXE));
	return ch;
}

void Bsp_Usart_Init(void)
{
	
  USART_InitTypeDef								usart;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
//	//usart1(遥控)
//	USART_DeInit(USART1);
//	USART_StructInit(&usart);
//	usart.USART_BaudRate							= 115200;
//	usart.USART_HardwareFlowControl		= USART_HardwareFlowControl_None;
//	usart.USART_Mode									= USART_Mode_Rx|USART_Mode_Tx;
//	usart.USART_Parity								= USART_Parity_No;
//	usart.USART_StopBits							= USART_StopBits_1;
//	usart.USART_WordLength						= USART_WordLength_8b;
//	USART_Init(USART1, &usart);
////	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
//  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
//	USART_Cmd(USART1, ENABLE);

	//usart3
	USART_DeInit(USART3);
	USART_StructInit(&usart);
	usart.USART_BaudRate							= 115200;
	usart.USART_HardwareFlowControl		= USART_HardwareFlowControl_None;
	usart.USART_Mode									= USART_Mode_Rx|USART_Mode_Tx;
	usart.USART_Parity								= USART_Parity_No;
	usart.USART_StopBits							= USART_StopBits_1;
	usart.USART_WordLength						= USART_WordLength_8b;
	USART_Init(USART3, &usart);
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART3, ENABLE);

//	USART_ClearFlag(USART1,USART_FLAG_IDLE);//防止接受不到第一个字符
//	USART_ClearFlag(USART3,USART_FLAG_IDLE);//防止接受不到第一个字符
}

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
char data_to_send[50];	//发送数据缓存
void usart_dma_upgrade(uint8_t* data,u8 len)
{
		if(DMA1_Stream3->NDTR)
    {
        return;
    }
    DMA_Cmd(DMA1_Stream3, DISABLE);                                     //关闭 DMA 传输
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}                 //确保 DMA 可以被设置
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);       //清空标志位
    DMA1_Stream3->M0AR = (uint32_t)data;              //设置数据
    DMA_SetCurrDataCounter(DMA1_Stream3, len);                          //数据传输量
    DMA_Cmd(DMA1_Stream3, ENABLE);
}

void Usart_DMASend(uint8_t *send,uint8_t len)//各种数据
{
	usart_dma_upgrade(send, len);
}

void USART1_IRQHandler(void)//接收遥控器值
{
    if (USART_GetITStatus(USART1, USART_IT_IDLE) == 1)
      {
				USART_ClearITPendingBit(USART1, USART_IT_IDLE);
				USART_ReceiveData(USART1);

      }
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) == 1)
      {
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
				USART_ReceiveData(USART3);
				    
      }
}

	



