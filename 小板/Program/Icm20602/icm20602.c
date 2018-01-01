#include "main.h"

uint8_t Icm20602_init(void)
{
	
	uint8_t initdata[10][2]={{ICM20602_PWR_MGMT_1,0x80},\
													 {1,1},\
													 {1,1},\
													};
	uint8_t mpu6500_id=0;
	
	uint8_t data=0,state=0;
	Icm20602_WriteByte(ICM20602_PWR_MGMT_1, 0x80);//陀螺仪重启
	delay_ms(100);
	Icm20602_ReadByte(ICM20602_WHO_AM_I,&mpu6500_id);
		if (mpu6500_id != ICM20602_WHO_AM_I_CONST)
		return 1; //校验失败，返回0xff
	delay_ms(1);
//	MPU6500_WriteByte(SPIx,MPU6500_PWR_MGMT_1, 0x03);
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx,MPU6500_INT_PIN_CFG, 0x10);//BYPASS模式
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx,MPU6500_INT_ENABLE, 0x01);//开启中断
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx,MPU6500_PWR_MGMT_2, 0x00);//108 
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx,MPU6500_SMPLRT_DIV, 0x00);//
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx, MPU6500_GYRO_CONFIG, 0x18);//2000dps
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx, MPU6500_ACCEL_CONFIG, 0x01);//4g
//	delay_ms(1);
//	MPU6500_ReadByte(SPIx,MPU6500_ACCEL_CONFIG_2,&data);	
//	data|=0x02;
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx, MPU6500_ACCEL_CONFIG_2, data);//rate=1khz
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx, MPU6500_CONFIG, 0x01);//DLPF_CFG=1 EXT GYROX
//	delay_ms(1);
//	MPU6500_ReadByte(SPIx, MPU6500_USER_CTRL, &state);
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx, MPU6500_USER_CTRL, state|0x10);
//	delay_ms(1);
//	MPU6500_ReadByte(SPIx, MPU6500_USER_CTRL, &state);
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx, MPU6500_USER_CTRL, state|0x20);
//	delay_ms(1);
//	MPU6500_WriteByte(SPIx,MPU6500_I2C_MST_DELAY_CTRL,0x10|0x20);
//  delay_ms(100);
	return 0;
}