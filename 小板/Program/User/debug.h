#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "stm32f4xx.h"
#include "monitor.h"


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

typedef struct
{
	struct{
		Function_t *ist8310;
		Function_t *icm20602;
		Function_t *ms5611;
		Function_t *ahrs;
		Function_t *controlLoop;
	}functiontTime;
}debug_t;
void Debug_Init(void);
void getFunctionTimeInit(void);
void getFunctionTime(Function_t *fun);

extern Function_t Ist8310_DataUpdate_t;
extern Function_t ICM20602_DataUpdate_t;
extern Function_t Ms5611_DataUpdate_t;
extern Function_t AHRS_Update_t;
extern Function_t controlLoop_t;
#endif

