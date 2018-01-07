#include "imu.h"
#include "main.h"
#include "ahrs.h"
#include "config.h"
#include "gpio.h"


void ICM20602_Update(void)
{

		#if USE_ICM20602
	cmd.Icm20602.Status=PinInt;
	if(cmd.Icm20602.Status==PinInt)
	{
		Icm20602_GetData(&cmd.Icm20602.Data);
		
//		Icm20602_GetData(&cmd.Icm20602.original);
//		IMU_Filter(&cmd.Icm20602.original,&cmd.Icm20602.Data);
		Icm20602_DataLimit(&cmd.Icm20602.Data);
		Icm20602_SetDataStatus(1);
		cmd.Icm20602.monitor.set(&cmd.Icm20602.monitor);
	}
	
		#endif
}



