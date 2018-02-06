#include "stm32f4xx.h"
#include "icm20602Int.h"
#include "icm20602Dri.h"
#include "ist8310Dri.h"
#include "ist8310.h"
#include "icm20602.h"
#include "ms5611Dri.h"
#include "ms5611.h"
#include "monitor.h" 
#include "config.h"
#include "delay.h"
#include "stdio.h"
#include "usart.h"
#include "nvic.h"
#include "gpio.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "i2c.h" 
#include "main.h" 
#include "spi.h"
#include "ahrs.h"
#include "control.h"
#include "imu.h"
#include "kalman.h"
#include "flash.h"
#include <stdio.h>
#include <string.h>
/*
作者 	;华北理工大学  范翔宇
QQ		:826011810
邮箱	:826011810@qq.com
*/
cmd_t cmd={0};

float initData[7][3];
void system_init(void)
{
	memcpy(initData,(void *)ADDR_FLASH_SECTOR_11,sizeof(initData));
	if(initData[6][0]==2018&&initData[6][1]==1&&initData[6][2]==2018)
	{
		memcpy(cmd.Icm20602.calibrate.ref,initData,sizeof(initData)-12);
		Icm20602_Set_Calibration_Status(1);
	}
	else
	{
		Icm20602_Set_Calibration_Status(0);
	}
	getFunctionTimeInit();
	kalmanInit();
	AHRS_Init(&cmd.ahrs);
	Monitor_Init();
  Bsp_NVIC_Init();
	Bsp_GPIO_Init();
	Bsp_DMA_Init();
	Bsp_Usart_Init();
	Bsp_IIC_Init();
#if	USE_ICM20602	
	Bsp_Spi_Init();
	Icm20602_init();
	Icm20602_OffsetInit();
	Icm20602IntInit();
#endif	
	
#if	USE_IST8310
	Ist8310_Init();
#endif

#if	USE_MS5611
	Ms5611_Init();
#endif

#if USE_TIM
	Bsp_Tim_Init();
  sysEnable();  
#endif  
}
uint32_t aa,bb;float cc;
int main(void)
{
  system_init();//系统初始化
	while (1)
	{
#if	USE_ICM20602
		if(Icm20602_GetStatus()==SPIInt)
		{
				if(Icm20602_GetIntStatus())
			{
				__disable_irq();
				getFunctionTime(&ICM20602_DataUpdate_t);
				__enable_irq();
			}
		}
#endif
#if	USE_IST8310
		if(IST8310_GetIntStatus(1))
		{
			__disable_irq();
			getFunctionTime(&Ist8310_DataUpdate_t);
			__enable_irq();
		}
#endif



  }
}
