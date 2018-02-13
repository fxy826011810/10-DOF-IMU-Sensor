#include "stm32f4xx.h"
#include "monitor.h" 
#include "config.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "imu.h"
#include "control.h"

//监视器重启
void Monitor_Reset(System_Monitor_t *mon)
{
	mon->count=0;
	mon->time=0;
}
//监视器初始化
void Monitor_Init(void)
{
	Monitor_Reset(&cmd.Icm20602->monitor);
	Monitor_Reset(&cmd.Ist8310->monitor);
	Monitor_Reset(&cmd.Ms5611->monitor);
}


//监视器计算
void Monitor_Calc(System_Monitor_t *mon)
{
	mon->count=mon->time;
	mon->time=0;
}
//设置监视器
void Monitor_Set(System_Monitor_t *mon)
{
	mon->time++;
}
//监视器更新
void Monitor_Update(void)
{
	Monitor_Calc(&cmd.Icm20602->monitor);
	Monitor_Calc(&cmd.Ist8310->monitor);
	Monitor_Calc(&cmd.Ms5611->monitor);
}



