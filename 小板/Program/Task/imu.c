#include "imu.h"
#include "main.h"
#include "ahrs.h"
#include "config.h"
#include "gpio.h"


void Imu_Update(void)
{
	LED(0);
		#if USE_ICM20602
		Icm20602_GetData(&cmd.Icm20602.Data);
		Icm20602_DataLimit(&cmd.Icm20602.Data);
		cmd.Icm20602.monitor.set(&cmd.Icm20602.monitor);
		
	
		#endif
	if(IST8310_GetStatus())
	{
		_9AxisAHRSupdate(&cmd.Icm20602.Data,&cmd.Ist8310.Data,&cmd.ahrs);
		IST8310_SetStatus(0);
	}
	else
	{
		_6AxisAHRSupdate(&cmd.Icm20602.Data,&cmd.ahrs);
	}
	LED(1);
}



