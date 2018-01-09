#include "stm32f4xx.h"
#include <stdio.h>
#include "usart.h"
#include "monitor.h"
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
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	//usart1(遥控)
	USART_DeInit(USART1);
	USART_StructInit(&usart);
	usart.USART_BaudRate							= 115200;
	usart.USART_HardwareFlowControl		= USART_HardwareFlowControl_None;
	usart.USART_Mode									= USART_Mode_Rx|USART_Mode_Tx;
	usart.USART_Parity								= USART_Parity_No;
	usart.USART_StopBits							= USART_StopBits_1;
	usart.USART_WordLength						= USART_WordLength_8b;
	USART_Init(USART1, &usart);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
//  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	USART_Cmd(USART1, ENABLE);

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
uint8_t data_to_send[50];	//发送数据缓存
void usart3_dma_upgrade(uint8_t* data,u8 len)
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

void usart1_dma_upgrade(uint8_t* data,u8 len)
{
		if(DMA2_Stream7->NDTR)
    {
        return;
    }
    DMA_Cmd(DMA2_Stream7, DISABLE);                                     //关闭 DMA 传输
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}                 //确保 DMA 可以被设置
    DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7);       //清空标志位
    DMA2_Stream7->M0AR = (uint32_t)data;              //设置数据
    DMA_SetCurrDataCounter(DMA2_Stream7, len);                          //数据传输量
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

void Usart_DMASend(USART_TypeDef* USARTx,uint8_t *send,uint8_t len)//各种数据
{
	if(USARTx==USART3)
	{
		usart3_dma_upgrade(send, len);
	}
	else if(USARTx==USART1)
	{
		usart1_dma_upgrade(send, len);
	}
}


void Usart_Send_Status(USART_TypeDef* USARTx,float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)//上位机接收的
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	if(USARTx==USART1)
	{
	usart1_dma_upgrade(data_to_send, _cnt);
	}
	else if(USARTx==USART3)
	{
		usart3_dma_upgrade(data_to_send, _cnt);
	}
}


void Usart_Send_Angle(USART_TypeDef* USARTx,float angle_rol, float angle_pit, float angle_yaw)//上位机接收的
{
	u8 _cnt=0;
	vs16 _temp;

	
	data_to_send[_cnt++]=0xAA;
	
	FT.F=angle_rol;
	data_to_send[_cnt++]=FT.U[0];
	data_to_send[_cnt++]=FT.U[1];
	data_to_send[_cnt++]=FT.U[2];
	data_to_send[_cnt++]=FT.U[3];
	
	FT.F=angle_pit;
	data_to_send[_cnt++]=FT.U[0];
	data_to_send[_cnt++]=FT.U[1];
	data_to_send[_cnt++]=FT.U[2];
	data_to_send[_cnt++]=FT.U[3];
	
	FT.F=angle_yaw;
	data_to_send[_cnt++]=FT.U[0];
	data_to_send[_cnt++]=FT.U[1];
	data_to_send[_cnt++]=FT.U[2];
	data_to_send[_cnt++]=FT.U[3];
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0xBB;
	if(USARTx==USART1)
	{
	usart1_dma_upgrade(data_to_send, _cnt);
	}
	else if(USARTx==USART3)
	{
		usart3_dma_upgrade(data_to_send, _cnt);
	}
}


void Usart_Send_Senser(USART_TypeDef* USARTx,s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)//各种数据
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
	sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	if(USARTx==USART1)
	{
	usart1_dma_upgrade(data_to_send, _cnt);
	}
	else if(USARTx==USART3)
	{
		usart3_dma_upgrade(data_to_send, _cnt);
	}
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

	



