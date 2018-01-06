#include "stm32f4xx.h"
#include "monitor.h" 
#include "config.h"
#include "main.h"
FormatTrans FT;//用于float类型转化为char[4]类型



System_Monitor_t System_Monitor=Monitor_default;
System_Monitor_t CanP_Monitor=Monitor_default;
System_Monitor_t CanY_Monitor=Monitor_default;
System_Monitor_t CanG_Monitor=Monitor_default;
System_Monitor_t Imu_Int_Monitor=Monitor_default;

void Monitor_Reset(System_Monitor_t *mon)
{
	mon->count=0;
	mon->time=0;
	mon->upd=&Monitor_Calc;
//	mon->rst=&Monitor_Reset;
}

void Monitor_Init(void)
{
	cmd.Icm20602.monitor.rst=&Monitor_Reset;
	cmd.Icm20602.monitor.rst(&cmd.Icm20602.monitor);
	
	cmd.Ist8310.monitor.rst=&Monitor_Reset;
	cmd.Ist8310.monitor.rst(&cmd.Ist8310.monitor);
	
	cmd.Ms5611.monitor.rst=&Monitor_Reset;
	cmd.Ms5611.monitor.rst(&cmd.Ms5611.monitor);
}



void Monitor_Calc(System_Monitor_t *mon)
{
	mon->count=mon->time;
	mon->time=0;
}

void Monitor_Update(void)
{
	cmd.Icm20602.monitor.upd(&cmd.Icm20602.monitor);
	cmd.Ist8310.monitor.upd(&cmd.Ist8310.monitor);
	cmd.Ms5611.monitor.upd(&cmd.Ms5611.monitor);
//	Monitor_Calc(&cmd.Icm20602.monitor);
//	Monitor_Calc(&cmd.Ist8310.monitor);
//	System_Monitor.upd(&System_Monitor);
//	
//	CanP_Monitor.upd(&CanP_Monitor);
//	CanY_Monitor.upd(&CanY_Monitor);
//	CanG_Monitor.upd(&CanG_Monitor);
//	Imu_Int_Monitor.upd(&Imu_Int_Monitor);
	
}
