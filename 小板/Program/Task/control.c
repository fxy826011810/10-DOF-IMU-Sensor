#include "stm32f4xx.h"
#include "icm20602.h"
#include "monitor.h" 
#include "control.h"
#include "config.h"
#include "ms5611.h" 
#include "usart.h"
#include "ahrs.h"
#include "main.h"
#include "tim.h"
#include "can.h"
#include <stdio.h>

void controlLoop(void)
{
		cmd.heart++;
		if(cmd.heart%1000==0)
	{
		Monitor_Update();//����������
		LED_HEAT();//led��
		/*
		�������д������� 
		�����Ǽ�����Ϊ0
		�����Ƕ�ȡ״̬Ϊ�ж϶�ȡ
		*/
		if(cmd.heart>=3000&&cmd.Icm20602->monitor.count==0&&PinInt==Icm20602_GetStatus())
		{
			Icm20602_SetStatus(SPIInt);
		}
	}
	if(Lost!=Icm20602_GetStatus())
	{
		getFunctionTime(&AHRS_Update_t);//���½Ƕ�
		if(AHRS_GetDataStatus())
		{
			Can_AngleSend(CAN1,cmd.Ahrs->angle);//can����
			if(cmd.heart%10==0)
			{
				Usart_Send_Angle(USART3,cmd.Ahrs->angle);//��������
				Usart_Send_Status(USART1,cmd.Ahrs->angle,0,0,0);//������λ������
			}
			if(cmd.heart%10==5)
			{
				Usart_Send_Senser(USART1,&cmd.Icm20602->Data.original,&cmd.Ist8310->Data.original,0);//������λ������
			}
			AHRS_SetDataStatus(0);//���ýǶ�����״̬Ϊ�ȴ�
		}
	}
	//��ѹ�ƶ�ȡ
	#if USE_MS5611
	if(cmd.heart%5==0)
	{
		__disable_irq();
		getFunctionTime(&Ms5611_DataUpdate_t);
		__enable_irq();
		Monitor_Set(&cmd.Ms5611->monitor);
	}
#endif
	
}






