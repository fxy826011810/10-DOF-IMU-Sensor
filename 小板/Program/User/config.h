#ifndef __CONFIG_H__
#define __CONFIG_H__
#include "stm32f4xx.h"
/*			I2C			*/
#define I2CX  I2C1
#define GPIO_AF_I2CX GPIO_AF_I2C1
#define RCC_APB1Periph_I2CX RCC_APB1Periph_I2C1
/*			I2C	IST8310		*/
#define I2CI_SCL_GPIO  GPIOB
#define I2CI_SCL_PIN  GPIO_Pin_10
#define I2CI_SCL_MODE  GPIO_Mode_OUT
#define I2CI_SCL_OTYPE  GPIO_OType_PP
#define I2CI_SCL_PinSource GPIO_PinSource10

#define I2CI_SDA_GPIO  GPIOB
#define I2CI_SDA_PIN  GPIO_Pin_11
#define I2CI_SDA_MODE  GPIO_Mode_OUT
#define I2CI_SDA_OTYPE  GPIO_OType_PP
#define I2CI_SDA_PinSource GPIO_PinSource11
/*			I2C	Ms5611		*/
#define I2CM_SCL_GPIO  GPIOA
#define I2CM_SCL_PIN  GPIO_Pin_8
#define I2CM_SCL_MODE  GPIO_Mode_OUT
#define I2CM_SCL_OTYPE  GPIO_OType_PP
#define I2CM_SCL_PinSource GPIO_PinSource8

#define I2CM_SDA_GPIO  GPIOC
#define I2CM_SDA_PIN  GPIO_Pin_9
#define I2CM_SDA_MODE  GPIO_Mode_OUT
#define I2CM_SDA_OTYPE  GPIO_OType_PP
#define I2CM_SDA_PinSource GPIO_PinSource9

/*			IST8310	RST INT		*/

#define IST8310_INT_GPIO  GPIOB
#define IST8310_INT_PIN  GPIO_Pin_0
#define IST8310_INT_MODE  GPIO_Mode_IN

#define IST8310_RST_GPIO  GPIOB
#define IST8310_RST_PIN  GPIO_Pin_1
#define IST8310_RST_MODE  GPIO_Mode_OUT
#define IST8310_RST_OTYPE  GPIO_OType_PP

/*			SPI	CS MISO MOSI SCLK INT*/
#define SPIX  SPI1
#define GPIO_AF_SPIX GPIO_AF_SPI1
#define RCC_APB2Periph_SPIX RCC_APB2Periph_SPI1
//中断
#define SPIX_IRQ_GPIO  GPIOA
#define SPIX_IRQ_PIN  GPIO_Pin_3
#define SPIX_IRQ_MODE  GPIO_Mode_IN
#define SPIX_IRQ_OTYPE  GPIO_OType_PP
//片选
#define SPIX_NSS_GPIO  GPIOA
#define SPIX_NSS_PIN  GPIO_Pin_4
#define SPIX_NSS_MODE  GPIO_Mode_OUT
#define SPIX_NSS_OTYPE  GPIO_OType_PP
//SCLK
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