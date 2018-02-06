#include "common.h"
#include "control.h"
#include "tim.h"
#include "main.h"
#include <string.h>
FormatTrans FT;//用于float类型转化为char[4]类型

Function_t Ist8310_DataUpdate_t;
Function_t ICM20602_DataUpdate_t;
Function_t Ms5611_DataUpdate_t;
Function_t AHRS_Update_t;
Function_t controlLoop_t;
static void getFunctionTimeStructInit(Function_t *fun)
{
	fun->funEnd=0;
	fun->funOperationTime=0;
	fun->funPeriodTime=0;
	fun->funStart=0;
	fun->funStartLast=0;
	fun->timeGet=&Get_Time_Micros;
	fun->Function=NULL;
}

void getFunctionTime(Function_t *fun)
{
	fun->funStart=fun->timeGet();
	fun->funPeriodTime=fun->funStart-fun->funStartLast;
	fun->funStartLast=fun->funStart;
	fun->Function();
	fun->funEnd=fun->timeGet();
	fun->funOperationTime=fun->funEnd-fun->funStart;
}

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


