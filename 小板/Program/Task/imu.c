#include "imu.h"
#include "main.h"
#include "kalman.h"
#include "config.h"

void AHRS_Update(void)
{
	kalmanUpdate();
	if(IST8310_GetStatus()==1&&IST8310_GetDataStatus()==1&&Icm20602_GetDataStatus()==1&&Icm20602_GetStatus()!=Lost)
	{
		_9AxisAHRSupdate(&cmd.Icm20602->Data.calc,&cmd.Ist8310->Data.calc,cmd.Ahrs);
		Icm20602_SetDataStatus(0);
		IST8310_SetDataStatus(0);
		AHRS_SetDataStatus(1);
	}
	else if(Icm20602_GetDataStatus()==1&&IST8310_GetDataStatus()==0&&Icm20602_GetStatus()!=Lost)
	{
		_6AxisAHRSupdate(&cmd.Icm20602->Data.calc,cmd.Ahrs);
		Icm20602_SetDataStatus(0);
		AHRS_SetDataStatus(1);
	}
	if(AHRS_GetDataStatus())
	{
		AHRS_Q_To_Angle(cmd.Ahrs);
	}
}	





