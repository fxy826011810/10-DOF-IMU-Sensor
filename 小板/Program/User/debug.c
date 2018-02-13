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

//Debug初始化
void Debug_Init(void)
{
	debug.functiontTime.ahrs=&AHRS_Update_t;
	debug.functiontTime.controlLoop=&controlLoop_t;
	debug.functiontTime.icm20602=&ICM20602_DataUpdate_t;
	debug.functiontTime.ist8310=&Ist8310_DataUpdate_t;
	debug.functiontTime.ms5611=&Ms5611_DataUpdate_t;
	getFunctionTimeInit();
}
//获取系统运行时间
static uint32_t getFunctionTimeMicros(void)
{
				return TIM2->CNT;
}

//结构体初始化
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
//获取函数运行周期和执行时间 us
void getFunctionTime(Function_t *fun)
{
	fun->funStart=fun->timeGet();
	fun->funPeriodTime=fun->funStart-fun->funStartLast;
	fun->funStartLast=fun->funStart;
	fun->Function();
	fun->funEnd=fun->timeGet();
	fun->funOperationTime=fun->funEnd-fun->funStart;
}
//初始化
void getFunctionTimeInit(void)
{
	Function_t function;
	uint16_t funSize;
	funSize=sizeof(function);
	
	getFunctionTimeStructInit(&function);
	
	function.Function=&ICM20602_DataUpdate;						//初始化icm任务
	memcpy(&ICM20602_DataUpdate_t,&function,funSize);
	
	function.Function=&Ist8310_DataUpdate;						//初始化磁力计任务
	memcpy(&Ist8310_DataUpdate_t,&function,funSize);
	
	function.Function=&Ms5611_DataUpdate;							//初始化气压计
	memcpy(&Ms5611_DataUpdate_t,&function,funSize);
	
	function.Function=&AHRS_Update;										//初始化姿态更新
	memcpy(&AHRS_Update_t,&function,funSize);
	
	function.Function=&controlLoop;										//初始化系统主任务
	memcpy(&controlLoop_t,&function,funSize);
}


