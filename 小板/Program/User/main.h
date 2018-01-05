#ifndef __MAIN_H__
#define __MAIN_H__

//#include <stdio.h>
//#include "stm32f4xx.h"
//#include "config.h"
//#include "nvic.h"

//#include "gpio.h"
//#include "monitor.h" 
//#include "delay.h"
//#include "usart.h"

//#include "can.h"

//#include "dma.h"
//#include "control.h"
//#include "tim.h"
//#include "i2c.h" 
//#include "ist8310.h"
//#include "ms5611.h"
//#include "spi.h"
//#include "icm20602Int.h"
//#include "icm20602Dri.h"
//#include "icm20602.h"
#include "stm32f4xx.h"
#include "icm20602.h"
#include "ist8310.h"
#include "ms5611.h"
typedef struct
{
	uint32_t heart;
	Icm20602Datadef icm20602;
	magDatadef			ist8310;
	Ms5611DataDef		ms5611;
}cmd_t;
extern cmd_t cmd;



//debug
//#define WHILE_DEBUG   0					//��ѭ������
//#define TIM_DEBUG			0					//timѭ��
//#define USE_TIM				1					//ʹ��tim
//#define USE_IST8310		1					//ʹ�ô�����
//#define USE_ICM20602	1					//ʹ��������
//#define USE_MS5611		0					//ʹ����ѹ��
#endif

