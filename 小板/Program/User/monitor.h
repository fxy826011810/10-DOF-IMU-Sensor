#ifndef __MONITOR_H__
#define __MONITOR_H__
#include "stm32f4xx.h"




typedef struct System_Monitor_t 
{
	int16_t count;
	int16_t time;
//	void(*upd)(struct System_Monitor_t *mon);
//	void(*rst)(struct System_Monitor_t *mon);
//	void(*set)(struct System_Monitor_t *mon);
}System_Monitor_t;


void Monitor_Init(void);
void Monitor_Update(void);
void Monitor_Reset(System_Monitor_t *mon);
void Monitor_Set(System_Monitor_t *mon);

#define Monitor_default \
{\
	0,\
	0,\
}

#endif
