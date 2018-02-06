#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "icm20602.h"
#include "ist8310.h"
#include "ms5611.h"
#include "monitor.h"
#include "ahrs.h"
#include "config.h"
#include "common.h"
#define	LED_HEAT() GPIOC->ODR^=GPIO_Pin_12
#define LED(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_12):GPIO_ResetBits(GPIOC,GPIO_Pin_12)

typedef struct
{
	uint32_t heart;
	uint32_t time,lasttime;
	uint16_t tim;
	struct 
	{
		Icm20602Status			status;					//imu选用中断或读取寄存器读取数据
		struct
		{
			uint8_t AccelOn;									//用于矫正加速度计
			uint8_t status;										//两个传感器的矫正状态
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
	}Icm20602;
	struct 
	{
		uint8_t						status;						//磁力计是否正常
		uint8_t						dataStatus;				//数据状态
		float Crossaxis[9];									//旋转矩阵
		struct
		{
			magDatadef				original;
			magDatadef				calc;
		}Data;
		System_Monitor_t 	monitor;	
	}Ist8310;
	struct 
	{
		Ms5611DataDef			Data;
		Ms5611Status			Status;
		System_Monitor_t 	monitor;
	}Ms5611;
	ahrs_t ahrs;
}cmd_t;

extern cmd_t cmd;


#endif

