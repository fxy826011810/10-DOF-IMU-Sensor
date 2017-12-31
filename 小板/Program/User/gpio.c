#include "main.h"

void Bsp_GPIO_Init(void)
{
	GPIO_InitTypeDef						gpio;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC| RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOA, ENABLE);

//led1
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode							= GPIO_Mode_OUT;
	gpio.GPIO_OType							= GPIO_OType_PP;
	gpio.GPIO_Pin							= GPIO_Pin_12;
	gpio.GPIO_PuPd							= GPIO_PuPd_UP;
	gpio.GPIO_Speed							= GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &gpio);

//usart1
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
//usart3
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_10| GPIO_Pin_11;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
//can1
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode							= GPIO_Mode_AF;
	gpio.GPIO_OType							= GPIO_OType_PP;
	gpio.GPIO_Pin							= GPIO_Pin_11 | GPIO_Pin_12;
	gpio.GPIO_PuPd							= GPIO_PuPd_UP;
	gpio.GPIO_Speed							= GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpio);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
	
	//陀螺仪中断
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode=SPIX_IRQ_MODE;
  gpio.GPIO_Pin = SPIX_IRQ_PIN;
  GPIO_Init(SPIX_IRQ_GPIO,&gpio);
	
//SPI(陀螺仪)
//片选
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode							= SPIX_NSS_MODE;
	gpio.GPIO_OType							= SPIX_NSS_OTYPE;
	gpio.GPIO_Pin								= SPIX_NSS_PIN;
	gpio.GPIO_PuPd							= GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed							= GPIO_Speed_50MHz;
	GPIO_Init(SPIX_NSS_GPIO, &gpio);
	GPIO_SetBits(SPIX_NSS_GPIO,SPIX_NSS_PIN);
	
	GPIO_StructInit(&gpio);
  gpio.GPIO_Mode							= SPIX_SCK_MODE;
	gpio.GPIO_OType							= SPIX_SCK_OTYPE;
	gpio.GPIO_Pin								= SPIX_SCK_PIN | SPIX_MISO_PIN	|	SPIX_MOSI_PIN;
	gpio.GPIO_PuPd							= GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed							= GPIO_Speed_50MHz;
	GPIO_Init(SPIX_SCK_GPIO, &gpio);
	GPIO_PinAFConfig(SPIX_SCK_GPIO, SPIX_SCK_PinSource, GPIO_AF_SPIX);
	GPIO_PinAFConfig(SPIX_MISO_GPIO, SPIX_MISO_PinSource, GPIO_AF_SPIX);
	GPIO_PinAFConfig(SPIX_MOSI_GPIO, SPIX_MOSI_PinSource, GPIO_AF_SPIX);
	
	//磁力计IIC
	//SCL
	ist8310IIC.gpio=I2CI_SCL_GPIO;
	ist8310IIC.scl_gpio_init.GPIO_Mode = I2CI_SCL_MODE;
	ist8310IIC.scl_gpio_init.GPIO_OType = I2CI_SCL_OTYPE;
	ist8310IIC.scl_gpio_init.GPIO_Pin = I2CI_SCL_PIN;
	ist8310IIC.scl_gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	ist8310IIC.scl_gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ist8310IIC.gpio, &ist8310IIC.scl_gpio_init);
	//SDA
	ist8310IIC.gpio=I2CI_SDA_GPIO;
	ist8310IIC.sda_gpio_init.GPIO_Mode = I2CI_SDA_MODE;
	ist8310IIC.sda_gpio_init.GPIO_OType = I2CI_SDA_OTYPE;
	ist8310IIC.sda_gpio_init.GPIO_Pin = I2CI_SDA_PIN;
	ist8310IIC.sda_gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	ist8310IIC.sda_gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ist8310IIC.gpio, &ist8310IIC.sda_gpio_init);
	//	GPIO_PinAFConfig(I2CI_SCL_GPIO, I2CI_SCL_PinSource, GPIO_AF_I2CI);
	//	GPIO_PinAFConfig(I2CI_SDA_GPIO, I2CI_SDA_PinSource, GPIO_AF_I2CI);
	//磁力计中断
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode=IST8310_INT_MODE;
  gpio.GPIO_Pin = IST8310_INT_PIN;
  GPIO_Init(IST8310_INT_GPIO,&gpio);
	//磁力计重启
	gpio.GPIO_Mode = IST8310_RST_MODE;
	gpio.GPIO_OType = IST8310_RST_OTYPE;
	gpio.GPIO_Pin = IST8310_RST_PIN;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(IST8310_RST_GPIO, &gpio);



	//气压计IIC
	//SCL
	ms5611IIC.gpio=I2CM_SCL_GPIO;
	ms5611IIC.scl_gpio_init.GPIO_Mode = I2CM_SCL_MODE;
	ms5611IIC.scl_gpio_init.GPIO_OType = I2CM_SCL_OTYPE;
	ms5611IIC.scl_gpio_init.GPIO_Pin = I2CM_SCL_PIN;
	ms5611IIC.scl_gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	ms5611IIC.scl_gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ms5611IIC.gpio, &ms5611IIC.scl_gpio_init);
	//SDA
	ms5611IIC.gpio=I2CM_SDA_GPIO;
	ms5611IIC.sda_gpio_init.GPIO_Mode = I2CM_SDA_MODE;
	ms5611IIC.sda_gpio_init.GPIO_OType = I2CM_SDA_OTYPE;
	ms5611IIC.sda_gpio_init.GPIO_Pin = I2CM_SDA_PIN;
	ms5611IIC.sda_gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	ms5611IIC.sda_gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ms5611IIC.gpio, &ms5611IIC.sda_gpio_init);


}





