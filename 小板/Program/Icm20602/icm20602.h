#ifndef __ICM20602_H__
#define __ICM20602_H__
#include "stm32f4xx.h"
#include "monitor.h"
#include "flash.h"
#define ICM20602_XG_OFFS_TC_H							0x04
#define ICM20602_XG_OFFS_TC_L							0x05
#define ICM20602_YG_OFFS_TC_H							0x07
#define ICM20602_YG_OFFS_TC_L							0x08
#define ICM20602_ZG_OFFS_TC_H							0x0A
#define ICM20602_ZG_OFFS_TC_L							0x0B
#define ICM20602_SELF_TEST_X_ACCEL					0x0D
#define ICM20602_SELF_TEST_Y_ACCEL					0x0E
#define ICM20602_SELF_TEST_Z_ACCEL					0x0F
#define ICM20602_XG_OFFS_USRH							0x13
#define ICM20602_XG_OFFS_USRL							0x14
#define ICM20602_YG_OFFS_USRH							0x15
#define ICM20602_YG_OFFS_USRL							0x16
#define ICM20602_ZG_OFFS_USRH							0x17
#define ICM20602_ZG_OFFS_USRL							0x18

#define ICM20602_SMPLRT_DIV								0x19
#define ICM20602_CONFIG										0x1A
#define ICM20602_GYRO_CONFIG								0x1B
#define ICM20602_ACCEL_CONFIG							0x1C
#define ICM20602_ACCEL_CONFIG_2						0x1D
#define ICM20602_LP_MODE_CFG								0x1E
#define ICM20602_ACCEL_WOM_X_THR						0x20
#define ICM20602_ACCEL_WOM_Y_THR						0x21
#define ICM20602_ACCEL_WOM_Z_THR						0x22
#define ICM20602_FIFO_EN										0x23

#define ICM20602_FSYNC_INT									0x36

#define ICM20602_INT_PIN_CFG								0x37
#define ICM20602_INT_ENABLE								0x38
#define ICM20602_FIFO_WM_INT_STATUS				0x39

#define ICM20602_INT_STATUS								0x3A

#define ICM20602_ACCEL_XOUT_H							0x3B
#define ICM20602_ACCEL_XOUT_L							0x3C
#define ICM20602_ACCEL_YOUT_H							0x3D
#define ICM20602_ACCEL_YOUT_L							0x3E
#define ICM20602_ACCEL_ZOUT_H							0x3F
#define ICM20602_ACCEL_ZOUT_L							0x40
#define ICM20602_TEMP_OUT_H								0x41
#define ICM20602_TEMP_OUT_L								0x42
#define ICM20602_GYRO_XOUT_H								0x43
#define ICM20602_GYRO_XOUT_L								0x44
#define ICM20602_GYRO_YOUT_H								0x45
#define ICM20602_GYRO_YOUT_L								0x46
#define ICM20602_GYRO_ZOUT_H								0x47
#define ICM20602_GYRO_ZOUT_L								0x48

#define ICM20602_SELF_TEST_X_GYRO					0x50
#define ICM20602_SELF_TEST_Y_GYRO					0x51
#define ICM20602_SELF_TEST_Z_GYRO					0x52
#define ICM20602_FIFO_WM_TH1								0x60
#define ICM20602_FIFO_WM_TH2								0x61
#define ICM20602_SIGNAL_PATH_RESET					0x68	
#define ICM20602_ACCEL_INTEL_CTRL					0x69

#define ICM20602_USER_CTRL									0x6A
#define ICM20602_PWR_MGMT_1								0x6B
#define ICM20602_PWR_MGMT_2								0x6C
#define ICM20602_I2C_IF										0x70
#define ICM20602_FIFO_COUNTH								0x72
#define ICM20602_FIFO_COUNTL								0x73
#define ICM20602_FIFO_R_W									0x74
#define ICM20602_WHO_AM_I									0x75
#define ICM20602_XA_OFFSET_H								0x77
#define ICM20602_XA_OFFSET_L								0x78
#define ICM20602_YA_OFFSET_H								0x7A
#define ICM20602_YA_OFFSET_L								0x7B
#define ICM20602_ZA_OFFSET_H								0x7D
#define ICM20602_ZA_OFFSET_L								0x7E
#define ICM20602_WHO_AM_I_CONST		(0X12)//WHO_AM_I编码


#define ICM20602LIMIT_MIN 2			//陀螺仪最小数据限制
#define calc_gyrotime 100				//陀螺仪矫正数据采样次数
#define check_gyrotime 10				//陀螺仪矫正数据采样次数
#define ACCMAX 7000							//加速度计矫正阈值
#define calc_acctime 500				//加速度计矫正数据采样次数

typedef struct Icm20602Datadef
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t temp;
	int16_t gx;
	int16_t gy;
	int16_t gz;
}Icm20602Datadef;

typedef enum
{
	PinInt,
	SPIInt,
	Lost,
}Icm20602Status;

typedef struct 
	{
		Icm20602Status			status;					//imu选用中断或读取寄存器读取数据
		struct
		{
			uint8_t AccelOn;									//用于矫正模式下矫正加速度 设置为1开始矫正
			uint8_t status;										//总矫正状态 1为矫正 0为未校正
			uint8_t accStatus;								//加速度计的矫正状态
			uint8_t gyroStatus;								//陀螺仪的的矫正状态
			float ref[6][3];									//用于存放加速度计的矫正原始数据
			float Crossaxis[9];								//加速度计旋转矩阵
			Icm20602Datadef	offset;						//用于存放矫正值
		}calibrate;
		struct
		{
			uint8_t							dataStatus;		//数据状态
			Icm20602Datadef			original;			//原始数据
			Icm20602Datadef 		calc;					//矫正数据
		}Data;
	System_Monitor_t 		monitor;					//记录imu帧率 更新频率为1hz
}Icm20602_t;
	
uint8_t Icm20602_init(void);
void Icm20602_OffsetInit(void);

void ICM20602_DataUpdate(void);

void Icm20602_GetData(Icm20602Datadef *icmdata);

void Icm20602DataFilter(Icm20602Datadef *data,Icm20602Datadef *out);

uint8_t Icm20602_GetIntStatus(void);

void Icm20602_SetDataStatus(uint8_t x);
uint8_t Icm20602_GetDataStatus(void);

Icm20602Status Icm20602_GetStatus(void);
void Icm20602_SetStatus(Icm20602Status x);

void Icm20602_Set_Calibration_Status(uint8_t  x);
uint8_t Icm20602_Get_Calibration_Status(void);

void Icm20602_openAccelCalibrate(uint8_t  x);
uint8_t Icm20602_isOpenAccelCalibrate(void);
#endif
