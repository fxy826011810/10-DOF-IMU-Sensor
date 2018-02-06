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


typedef struct Function_t
{
	uint32_t funStart;				//������ʼʱ��
	uint32_t funStartLast;		//������һ�ο�ʼʱ��
	uint32_t funEnd;					//����ִ�н���ʱ��
	uint16_t funPeriodTime;		//����ִ������
	uint16_t funOperationTime;//��������ʱ��
	void (* Function)(void);
	uint32_t (* timeGet)(void);
}Function_t;

void getFunctionTimeInit(void);
void getFunctionTime(Function_t *fun);

extern Function_t Ist8310_DataUpdate_t;
extern Function_t ICM20602_DataUpdate_t;
extern Function_t Ms5611_DataUpdate_t;
extern Function_t AHRS_Update_t;
extern Function_t controlLoop_t;
//struct calibration_t {
//	float	x_offset;
//	float	x_scale;
//	float	y_offset;
//	float	y_scale;
//	float	z_offset;
//	float	z_scale;
//};
#endif

