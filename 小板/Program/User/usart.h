#ifndef __USART_H
#define __USART_H
#include "stm32f4xx.h"
#include "main.h"


#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
void Bsp_Usart_Init(void);
void Usart_DMASend(USART_TypeDef* USARTx,uint8_t *send,uint8_t len);
void Usart_Send_Status(USART_TypeDef* USARTx,float angle[3], s32 alt, u8 fly_model, u8 armed);//上位机接收的
void Usart_Send_Senser(USART_TypeDef* USARTx,Icm20602Datadef *icm,magDatadef *ist,s32 bar);
void Usart_Send_Angle(USART_TypeDef* USARTx,float angle[3]);
#endif

