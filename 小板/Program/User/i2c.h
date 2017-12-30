#ifndef __I2C_H
#define __I2C_H
#include "stm32f4xx.h"

#define SDA_IN()  {\
	GPIO_InitTypeDef						gpio;\
	gpio.GPIO_Mode							= GPIO_Mode_IN;\
	gpio.GPIO_Pin								= I2CX_SDA_PIN;\
	gpio.GPIO_PuPd							= GPIO_PuPd_NOPULL;\
	GPIO_Init(I2CX_SDA_GPIO, &gpio);\
} //PB9输出模式
#define SDA_OUT() {\
	GPIO_InitTypeDef						gpio;\
	gpio.GPIO_Mode							= GPIO_Mode_OUT;\
	gpio.GPIO_OType							= GPIO_OType_PP;\
	gpio.GPIO_Pin								= I2CX_SDA_PIN;\
	gpio.GPIO_PuPd							= GPIO_PuPd_NOPULL;\
	gpio.GPIO_Speed							= GPIO_Speed_50MHz;\
	GPIO_Init(I2CX_SDA_GPIO, &gpio);\
} //PB9输出模式

//IO操作函数	 

#define IIC_SCL(x) x ? GPIO_SetBits(I2CX_SCL_GPIO,I2CX_SCL_PIN):GPIO_ResetBits(I2CX_SCL_GPIO,I2CX_SCL_PIN)
#define IIC_SDA(x) x ? GPIO_SetBits(I2CX_SDA_GPIO,I2CX_SDA_PIN):GPIO_ResetBits(I2CX_SDA_GPIO,I2CX_SDA_PIN)
#define READ_SDA      GPIO_ReadInputDataBit(I2CX_SDA_GPIO,I2CX_SDA_PIN)//输入SDA 

//typedef	
void	Bsp_IIC_Init(void);

uint8_t IIC_Start(void);
void	IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void	IIC_Ack(void);
//void	IIC_NAck(void);

void	IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t	IIC_WriteByte(uint8_t Addr,uint8_t reg,uint8_t Data);
uint8_t IIC_ReadByte(uint8_t Addr, uint8_t reg, uint8_t *pbuffer);
uint8_t	IIC_Read(uint8_t Addr, uint8_t reg,uint8_t *pbuffer, uint8_t len);
uint8_t	IIC_Write(uint8_t Addr,uint8_t reg,uint8_t *Data,uint8_t len);
uint8_t IIC_Read_x(uint8_t Addr, uint8_t reg, uint8_t *pbuffer, uint8_t len);
uint8_t IIC_Write_x(uint8_t Addr, uint8_t reg, uint8_t *Data, uint8_t len);
uint8_t IIC_Read1(uint8_t Addr,uint8_t reg,uint8_t len,uint8_t *pbuffer);
uint8_t IIC_Write1(uint8_t Addr,uint8_t reg,uint8_t len,uint8_t *Data);

#endif

