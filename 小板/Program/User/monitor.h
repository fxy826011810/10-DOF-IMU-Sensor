#ifndef __MONITOR_H__
#define __MONITOR_H__
#include "stm32f4xx.h"
typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FormatTrans;
extern FormatTrans FT;

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}

typedef struct System_Monitor_t 
{
	int16_t count;
	int16_t time;
	void(*upd)(struct System_Monitor_t *mon);
}System_Monitor_t;

extern System_Monitor_t ICM20602_Monitor;
extern System_Monitor_t Imu_Int_Monitor;
extern System_Monitor_t MAG_Monitor;
extern System_Monitor_t System_Monitor;
extern System_Monitor_t CanP_Monitor;
extern System_Monitor_t CanY_Monitor;
extern System_Monitor_t CanG_Monitor;

void Monitor_Calc(System_Monitor_t *mon);
void Monitor_Update(void);
#define Monitor_default \
{\
	0,\
	0,\
	&Monitor_Calc,\
}

#endif
