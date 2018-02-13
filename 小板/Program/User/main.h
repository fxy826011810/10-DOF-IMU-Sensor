#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "icm20602.h"
#include "ist8310.h"
#include "ms5611.h"
#include "monitor.h"
#include "ahrs.h"
#include "config.h"
#include "debug.h"

#define	LED_HEAT() GPIOC->ODR^=GPIO_Pin_12
#define LED(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_12):GPIO_ResetBits(GPIOC,GPIO_Pin_12)

typedef struct
{
	uint32_t heart;

	Icm20602_t *Icm20602;

	Ist8310_t *Ist8310;

	Ms5611_t *Ms5611;
	
	ahrs_t *Ahrs;
	
	debug_t *Debug;
	
}cmd_t;

extern cmd_t cmd;


#endif

