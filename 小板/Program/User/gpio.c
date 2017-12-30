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
//	//陀螺仪中断
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode=SPIX_IRQ_MODE;
  gpio.GPIO_Pin = SPIX_IRQ_PIN;
  GPIO_Init(SPIX_IRQ_GPIO,&gpio);
//IIC
	//SCL
	gpio.GPIO_Mode = I2CX_SCL_MODE;
	gpio.GPIO_OType = I2CX_SCL_OTYPE;
	gpio.GPIO_Pin = I2CX_SCL_PIN;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(I2CX_SCL_GPIO, &gpio);
	//SDA
	gpio.GPIO_Mode = I2CX_SDA_MODE;
	gpio.GPIO_OType = I2CX_SDA_OTYPE;
	gpio.GPIO_Pin = I2CX_SDA_PIN;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(I2CX_SDA_GPIO, &gpio);
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
//	GPIO_PinAFConfig(I2CX_SCL_GPIO, I2CX_SCL_PinSource, GPIO_AF_I2CX);
//	GPIO_PinAFConfig(I2CX_SDA_GPIO, I2CX_SDA_PinSource, GPIO_AF_I2CX);

}





