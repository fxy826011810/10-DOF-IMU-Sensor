#include "main.h"
FormatTrans FT;//用于float类型转化为char[4]类型


System_Monitor_t IMURec_Monitor=Monitor_default;
System_Monitor_t MAG_Monitor=Monitor_default;
System_Monitor_t System_Monitor=Monitor_default;
System_Monitor_t CanP_Monitor=Monitor_default;
System_Monitor_t CanY_Monitor=Monitor_default;
System_Monitor_t CanG_Monitor=Monitor_default;
System_Monitor_t Imu_Int_Monitor=Monitor_default;

void Monitor_Init(void)
{

}

void Monitor_Calc(System_Monitor_t *mon)
{
	mon->count=mon->time;
	mon->time=0;
}

void Monitor_Update(void)
{
	IMURec_Monitor.upd(&IMURec_Monitor);
	
	MAG_Monitor.upd(&MAG_Monitor);
	System_Monitor.upd(&System_Monitor);
	
	CanP_Monitor.upd(&CanP_Monitor);
	CanY_Monitor.upd(&CanY_Monitor);
	CanG_Monitor.upd(&CanG_Monitor);
	Imu_Int_Monitor.upd(&Imu_Int_Monitor);
	
}
