#ifndef __USART_H
#define __USART_H
#include "stm32f4xx.h"
void Bsp_Usart_Init(void);
void Usart_DMASend(USART_TypeDef* USARTx,uint8_t *send,uint8_t len);
void Usart_Send_Status(USART_TypeDef* USARTx,float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void Usart_Send_Senser(USART_TypeDef* USARTx,s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar);
void Usart_Send_Angle(USART_TypeDef* USARTx,float angle_rol, float angle_pit, float angle_yaw);
#endif

