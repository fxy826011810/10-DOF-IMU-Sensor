#include "main.h"


void system_init(void)
{
  Bsp_NVIC_Init();
	Bsp_GPIO_Init();
	Bsp_DMA_Init();
	Bsp_Usart_Init();
	Bsp_IIC_Init();
	Bsp_Spi_Init();
	Bsp_Tim_Init();
	delay_ms(1000);
	Icm20602_init();
	Icm20602IntInit();
	Ist8310_Init();
	Ms5611_Init();
  sysEnable();    
}

int main(void)
{
	uint8_t id;
  system_init();//系统初始化
	while (1)
	{
		if(IST8310_GetIntData())
		{
		IST8310_GetData(&mag);
		MAG_Monitor.time++;
		}
//		delay_ms(100);
//		Ms5611_ReadADC();

		
	//LED_HEAT();
  }
}
