#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "icm20602.h"
#include "ist8310.h"
#include "ms5611.h"
#include "monitor.h"
#include "ahrs.h"


#define	LED_HEAT() GPIOC->ODR^=GPIO_Pin_12
#define LED(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_12):GPIO_ResetBits(GPIOC,GPIO_Pin_12)

typedef struct
{
	uint32_t heart;
	struct 
	{
	uint8_t							dataStatus;
		struct
		{
			Icm20602Datadef			original;
			Icm20602Datadef 		calc;
		}Data;
	
	Icm20602Status			status;
	System_Monitor_t 		monitor;	
	}Icm20602;
	struct 
	{
		
		uint8_t						status;
		uint8_t						dataStatus;
		float Crossaxis[9];
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

