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
//#define WHILE_DEBUG   0					//主循环调试
//#define TIM_DEBUG			0					//tim循环
//#define USE_TIM				1					//使用tim
//#define USE_IST8310		1					//使用磁力计
//#define USE_ICM20602	1					//使用陀螺仪
//#define USE_MS5611		0					//使用气压计
#endif

