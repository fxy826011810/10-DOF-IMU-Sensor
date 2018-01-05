#include "stm32f4xx.h"


#include "spi.h"
#include "icm20602Int.h"
#include "icm20602Dri.h"
#include "icm20602.h"


#include "ist8310.h"


#include "ms5611.h"

#include "control.h"
#include "monitor.h" 
#include "config.h"
#include "delay.h"
#include <stdio.h>
//#include "usart.h"
#include "nvic.h"
#include "gpio.h"
//#include "can.h"
//#include "dma.h"
#include "tim.h"
#include "i2c.h" 
#include "main.h" 

cmd_t cmd;

uint32_t ms5611_temp;
uint32_t ms5611_pressure;
void system_init(void)
{
  Bsp_NVIC_Init();
	Bsp_GPIO_Init();
//	Bsp_DMA_Init();
//	Bsp_Usart_Init();
//	Bsp_IIC_Init();
	delay_ms(1000);

#if	USE_ICM20602	
	Bsp_Spi_Init();
	Icm20602_init();
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
#if USE_ICM20602
		if(Icm20602_GetIntData())
		{
			LED(0);
		Icm20602_GetData(&cmd.icm20602);
		ICM20602_Monitor.time++;
			LED(1);
		}
#endif
#if	USE_IST8310
		if(IST8310_GetIntData())
		{
		IST8310_GetData(&cmd.ist8310);
		MAG_Monitor.time++;
		}
#endif
#if USE_MS5611
		Ms5611_ReadD(Convert_D2_256,&cmd.ms5611.temp);
		Ms5611_ReadD(Convert_D1_256,&cmd.ms5611.pressure);
#endif		
#if WHILE_DEBUG
		LED_HEAT();
		delay_ms(1);
#endif

  }
}
