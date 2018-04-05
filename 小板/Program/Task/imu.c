#include "imu.h"
#include "main.h"
#include "kalman.h"
#include "config.h"

void AHRS_Update(void)
{
	kalmanUpdate();
	if(Icm20602_GetDataStatus()==1)
	{
		if(IST8310_GetDataStatus()==1)
		{
			_9AxisAHRSupdate(&cmd.Icm20602->Data.calc,&cmd.Ist8310->Data.calc,cmd.Ahrs);
			Icm20602_SetDataStatus(0);
			IST8310_SetDataStatus(0);
			AHRS_SetDataStatus(1);
		}
		else
		{
			_6AxisAHRSupdate(&cmd.Icm20602->Data.calc,cmd.Ahrs);
			Icm20602_SetDataStatus(0);
			AHRS_SetDataStatus(1);
		}
		AHRS_Q_To_Angle(cmd.Ahrs);
	}
	else
	{
		AHRS_SetDataStatus(0);
	}
}	





