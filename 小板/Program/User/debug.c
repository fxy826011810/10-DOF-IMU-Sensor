#include "debug.h"
#include "control.h"
#include "tim.h"
#include "main.h"
#include "imu.h"
#include <string.h>

debug_t debug;

Function_t Ist8310_DataUpdate_t;
Function_t ICM20602_DataUpdate_t;
Function_t Ms5611_DataUpdate_t;
Function_t AHRS_Update_t;
Function_t controlLoop_t;

//Debug��ʼ��
void Debug_Init(void)
{
	debug.functiontTime.ahrs=&AHRS_Update_t;
	debug.functiontTime.controlLoop=&controlLoop_t;
	debug.functiontTime.icm20602=&ICM20602_DataUpdate_t;
	debug.functiontTime.ist8310=&Ist8310_DataUpdate_t;
	debug.functiontTime.ms5611=&Ms5611_DataUpdate_t;
	getFunctionTimeInit();
}
//��ȡϵͳ����ʱ��
static uint32_t getFunctionTimeMicros(void)
{
				return TIM2->CNT;
}

//�ṹ���ʼ��
static void getFunctionTimeStructInit(Function_t *fun)
{
	fun->funEnd=0;
	fun->funOperationTime=0;
	fun->funPeriodTime=0;
	fun->funStart=0;
	fun->funStartLast=0;
	fun->timeGet=&getFunctionTimeMicros;
	fun->Function=NULL;
}
//��ȡ�����������ں�ִ��ʱ�� us
void getFunctionTime(Function_t *fun)
{
	fun->funStart=fun->timeGet();
	fun->funPeriodTime=fun->funStart-fun->funStartLast;
	fun->funStartLast=fun->funStart;
	fun->Function();
	fun->funEnd=fun->timeGet();
	fun->funOperationTime=fun->funEnd-fun->funStart;
}
//��ʼ��
void getFunctionTimeInit(void)
{
	Function_t function;
	uint16_t funSize;
	funSize=sizeof(function);
	
	getFunctionTimeStructInit(&function);
	
	function.Function=&ICM20602_DataUpdate;						//��ʼ��icm����
	memcpy(&ICM20602_DataUpdate_t,&function,funSize);
	
	function.Function=&Ist8310_DataUpdate;						//��ʼ������������
	memcpy(&Ist8310_DataUpdate_t,&function,funSize);
	
	function.Function=&Ms5611_DataUpdate;							//��ʼ����ѹ��
	memcpy(&Ms5611_DataUpdate_t,&function,funSize);
	
	function.Function=&AHRS_Update;										//��ʼ����̬����
	memcpy(&AHRS_Update_t,&function,funSize);
	
	function.Function=&controlLoop;										//��ʼ��ϵͳ������
	memcpy(&controlLoop_t,&function,funSize);
}


