#include "main.h"

uint8_t Icm20602_init(void)
{
	
	uint8_t i,initdata[10][2]={ {ICM20602_PWR_MGMT_1,0x80},\
															{ICM20602_USER_CTRL,0x01},\
															{ICM20602_SMPLRT_DIV,1},\
															{ICM20602_GYRO_CONFIG,0x18},
															{ICM20602_ACCEL_CONFIG,0x08},\
															{ICM20602_ACCEL_CONFIG_2,0x03},\
															{ICM20602_CONFIG,0x03},\
															{ICM20602_INT_STATUS,0x01},\
															{ICM20602_PWR_MGMT_2,0x00},\
															{ICM20602_I2C_IF,0x40},\
														};
	uint8_t mpu6500_id=0;										
	Icm20602_ReadByte(ICM20602_WHO_AM_I,&mpu6500_id);
	if (mpu6500_id != ICM20602_WHO_AM_I_CONST)
	return 1; //–£—È ß∞‹£¨∑µªÿ0xff													
	for(i=0;i<=8;i++)
	{
	Icm20602_WriteByte(initdata[i][0], initdata[i][1]);//Õ”¬›“«÷ÿ∆Ù
	delay_ms(10);
	}
	return 0;
}