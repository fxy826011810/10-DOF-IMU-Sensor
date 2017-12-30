#ifndef __CONFIG_H__
#define __CONFIG_H__
#include "stm32f4xx.h"
/*			I2C			*/
#define I2CX  I2C1
#define GPIO_AF_I2CX GPIO_AF_I2C1
#define RCC_APB1Periph_I2CX RCC_APB1Periph_I2C1

#define I2CX_SCL_GPIO  GPIOB
#define I2CX_SCL_PIN  GPIO_Pin_10
#define I2CX_SCL_MODE  GPIO_Mode_OUT
#define I2CX_SCL_OTYPE  GPIO_OType_PP
#define I2CX_SCL_PinSource GPIO_PinSource10

#define I2CX_SDA_GPIO  GPIOB
#define I2CX_SDA_PIN  GPIO_Pin_11
#define I2CX_SDA_MODE  GPIO_Mode_OUT
#define I2CX_SDA_OTYPE  GPIO_OType_PP
#define I2CX_SDA_PinSource GPIO_PinSource11
/*			IST8310			*/

#define IST8310_INT_GPIO  GPIOB
#define IST8310_INT_PIN  GPIO_Pin_0
#define IST8310_INT_MODE  GPIO_Mode_IN

#define IST8310_RST_GPIO  GPIOB
#define IST8310_RST_PIN  GPIO_Pin_1
#define IST8310_RST_MODE  GPIO_Mode_OUT
#define IST8310_RST_OTYPE  GPIO_OType_PP

/*			SPI			*/
#define SPIX  SPI1
#define GPIO_AF_SPIX GPIO_AF_SPI1
#define RCC_APB2Periph_SPIX RCC_APB2Periph_SPI1
//中断
#define SPIX_IRQ_GPIO  GPIOA
#define SPIX_IRQ_PIN  GPIO_Pin_3
#define SPIX_IRQ_MODE  GPIO_Mode_IN

//片选
#define SPIX_NSS_GPIO  GPIOA
#define SPIX_NSS_PIN  GPIO_Pin_4
#define SPIX_NSS_MODE  GPIO_Mode_OUT
#define SPIX_NSS_OTYPE  GPIO_OType_PP

#define SPIX_SCK_GPIO  GPIOA
#define SPIX_SCK_PIN  GPIO_Pin_5
#define SPIX_SCK_MODE  GPIO_Mode_AF
#define SPIX_SCK_OTYPE  GPIO_OType_PP
#define SPIX_SCK_PinSource GPIO_PinSource5
//从入主出
#define SPIX_MISO_GPIO  GPIOA
#define SPIX_MISO_PIN  GPIO_Pin_6
#define SPIX_MISO_MODE  GPIO_Mode_AF
#define SPIX_MISO_OTYPE  GPIO_OType_PP
#define SPIX_MISO_PinSource GPIO_PinSource6
//主入从出
#define SPIX_MOSI_GPIO  GPIOA
#define SPIX_MOSI_PIN  GPIO_Pin_7
#define SPIX_MOSI_MODE  GPIO_Mode_AF
#define SPIX_MOSI_OTYPE  GPIO_OType_PP
#define SPIX_MOSI_PinSource GPIO_PinSource7


#endif