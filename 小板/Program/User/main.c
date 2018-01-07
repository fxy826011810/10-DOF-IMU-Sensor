#include "stm32f4xx.h"
#include "icm20602Int.h"
#include "icm20602Dri.h"
#include "icm20602.h"
#include "ist8310.h"
#include "ms5611.h"
#include "control.h"
#include "monitor.h" 
#include "config.h"
#include "delay.h"
#include "stdio.h"
#include "usart.h"
#include "nvic.h"
#include "gpio.h"
#include "ahrs.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "i2c.h" 
#include "main.h" 
#include "spi.h"
cmd_t cmd={0};
char saychar[17];uint8_t testchar[]={"I love you\r\n"};
uint16_t saycharsize=0,i;

uint32_t ms5611_temp;
uint32_t ms5611_pressure;
void system_init(void)
{
	AHRS_Init(&cmd.ahrs);
	Monitor_Init();
  Bsp_NVIC_Init();
	Bsp_GPIO_Init();
	Bsp_DMA_Init();
	Bsp_Usart_Init();
//	Bsp_IIC_Init();
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
		if(cmd.Icm20602.Status==SPIInt)
		{
				if(Icm20602_GetIntData())
		{
			__disable_irq();
		Icm20602_GetData(&cmd.Icm20602.Data);
		Icm20602_SetDataStatus(1);
		cmd.Icm20602.monitor.set(&cmd.Icm20602.monitor);;
			__enable_irq();
		}
		}
#endif
#if	USE_IST8310
		if(IST8310_GetIntData(1))
		{
			__disable_irq();
			IST8310_GetData(&cmd.Ist8310.Data);
			IST8310_SetStatus(1);
			cmd.Ist8310.monitor.set(&cmd.Ist8310.monitor);
			__enable_irq();
		}
#endif

#if WHILE_DEBUG
		LED_HEAT();
#endif

  }
}
