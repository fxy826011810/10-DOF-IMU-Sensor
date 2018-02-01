#ifndef __COMMON_H__
#define __COMMON_H__

#include "stm32f4xx.h"

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}

typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FormatTrans;

extern FormatTrans FT;

//struct calibration_t {
//	float	x_offset;
//	float	x_scale;
//	float	y_offset;
//	float	y_scale;
//	float	z_offset;
//	float	z_scale;
//};
#endif

