#include "main.h"


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
#if	USE_IST8310
		if(IST8310_GetIntData())
		{
		IST8310_GetData(&mag);
		MAG_Monitor.time++;
		}
#endif
		
#if WHILE_DEBUG
		LED_HEAT();
		delay_ms(1);
#endif

  }
}
