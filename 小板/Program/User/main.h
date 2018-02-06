#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "icm20602.h"
#include "ist8310.h"
#include "ms5611.h"
#include "monitor.h"
#include "ahrs.h"
#include "config.h"
#include "common.h"
#define	LED_HEAT() GPIOC->ODR^=GPIO_Pin_12
#define LED(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_12):GPIO_ResetBits(GPIOC,GPIO_Pin_12)

typedef struct
{
	uint32_t heart;
	uint32_t time,lasttime;
	uint16_t tim;
	struct 
	{
		Icm20602Status			status;					//imuѡ���жϻ��ȡ�Ĵ�����ȡ����
		struct
		{
			uint8_t AccelOn;									//���ڽ������ٶȼ�
			uint8_t status;										//�����������Ľ���״̬
			float ref[6][3];									//���ڴ�ż��ٶȼƵĽ���ԭʼ����
			float Crossaxis[9];								//���ٶȼ���ת����
			Icm20602Datadef	offset;						//���ڴ�Ž���ֵ
		}calibrate;
		struct
		{
			uint8_t							dataStatus;		//����״̬
			Icm20602Datadef			original;			//ԭʼ����
			Icm20602Datadef 		calc;					//��������
		}Data;
	System_Monitor_t 		monitor;					//��¼imu֡�� ����Ƶ��Ϊ1hz
	}Icm20602;
	struct 
	{
		uint8_t						status;						//�������Ƿ�����
		uint8_t						dataStatus;				//����״̬
		float Crossaxis[9];									//��ת����
		struct
		{
			magDatadef				original;
			magDatadef				calc;
		}Data;
		System_Monitor_t 	monitor;	
	}Ist8310;
	struct 
	{
		Ms5611DataDef			Data;
		Ms5611Status			Status;
		System_Monitor_t 	monitor;
	}Ms5611;
	ahrs_t ahrs;
}cmd_t;

extern cmd_t cmd;


#endif

