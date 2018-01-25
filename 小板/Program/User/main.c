#include "stm32f4xx.h"
#include "icm20602Int.h"
#include "icm20602Dri.h"
#include "icm20602.h"
#include "ist8310.h"
#include "ist8310Dri.h"
#include "ms5611.h"
#include "ms5611Dri.h"
#include "monitor.h" 
#include "config.h"
#include "delay.h"
#include "stdio.h"
#include "usart.h"
#include "nvic.h"
#include "gpio.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "i2c.h" 
#include "main.h" 
#include "spi.h"
#include "ahrs.h"
#include "control.h"
#include "imu.h"
#include "kalman.h"

cmd_t cmd={0};

uint32_t ms5611_temp;
uint32_t ms5611_pressure;
void system_init(void)
{
	kalmanInit();
	AHRS_Init(&cmd.ahrs);
	Monitor_Init();
  Bsp_NVIC_Init();
	Bsp_GPIO_Init();
	Bsp_DMA_Init();
	Bsp_Usart_Init();

	delay_ms(1000);

#if	USE_ICM20602	
	Bsp_Spi_Init();
	Icm20602_init();
	
	Icm20602_Calc();
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
				ICM20602_DataUpdate();
				__enable_irq();
			}
		}
#endif
#if	USE_IST8310
		if(IST8310_GetIntStatus(1))
		{
			__disable_irq();
				Ist8310_DataUpdate();
			__enable_irq();
		}
#endif

#if WHILE_DEBUG
		LED_HEAT();
#endif

  }
}
