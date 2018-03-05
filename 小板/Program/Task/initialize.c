#include "initialize.h"
#include "flash.h"
#include <string.h>
#include "main.h"
extern cmd_t cmd;
extern Ms5611_t ms5611;
extern Icm20602_t icm20602;
extern Ist8310_t ist8310;
extern ahrs_t ahrs;
extern debug_t debug;


static void cmdOffsetDataInit(void)
{
	float initData[7][3];
	memcpy(initData,(void *)ADDR_FLASH_SECTOR_11,sizeof(initData));
	if(initData[6][0]==2018&&initData[6][1]==1&&initData[6][2]==2018)
	{
		memcpy(cmd.Icm20602->calibrate.ref,initData,sizeof(initData)-12);
		Icm20602_Set_Calibration_Status(1);
	}
	else
	{
		Icm20602_Set_Calibration_Status(0);
	}
}
void cmd_init(void)
{
	cmd.Ms5611=&ms5611;
	cmd.Icm20602=&icm20602;
	cmd.Ist8310=&ist8310;
	cmd.Ahrs=&ahrs;
	cmd.Debug=&debug;
	cmdOffsetDataInit();
}












