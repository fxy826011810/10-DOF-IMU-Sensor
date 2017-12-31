#ifndef __I2C_H
#define __I2C_H
#include "stm32f4xx.h"

typedef struct SimIIC_Typedef
{
	uint8_t Addr;
	GPIO_TypeDef *gpio;
	GPIO_InitTypeDef	sda_gpio_init;
	GPIO_InitTypeDef	scl_gpio_init;
	void(*WriteByte)(struct SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data);
	void(*WriteBytes)(struct SimIIC_Typedef *simiic, uint8_t reg, uint8_t *Data, uint8_t len);
	void(*ReadByte)(struct SimIIC_Typedef *simiic,uint8_t reg, uint8_t *pbuffer);
	void(*ReadBytes)(struct SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len);
}SimIIC_Typedef;

extern SimIIC_Typedef ist8310IIC;
extern SimIIC_Typedef ms5611IIC;

void Bsp_IIC_Init(void);

//uint8_t IIC_WriteByte(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data);//写单个字节
//uint8_t IIC_ReadByte(SimIIC_Typedef *simiic,uint8_t reg, uint8_t *pbuffer);//读单个字节
//uint8_t IIC_Read(SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len);//读多个字节
//uint8_t IIC_Write(SimIIC_Typedef *simiic, uint8_t reg, uint8_t *Data, uint8_t len);//写多个字节

void IIC_SCL(SimIIC_Typedef *simiic,uint8_t x);
void IIC_SDA(SimIIC_Typedef *simiic,uint8_t x);
uint8_t READ_SDA(SimIIC_Typedef *simiic);
void SDA_IN(SimIIC_Typedef *simiic);
void SDA_OUT(SimIIC_Typedef *simiic);
uint8_t IIC_Start(SimIIC_Typedef *simiic);
void IIC_Stop(SimIIC_Typedef *simiic);
uint8_t IIC_Wait_Ack(SimIIC_Typedef *simiic);
void IIC_NAck(SimIIC_Typedef *simiic);
void IIC_Ack(SimIIC_Typedef *simiic);
void IIC_Send_Byte(SimIIC_Typedef *simiic,uint8_t txd);
uint8_t IIC_Read_Byte(SimIIC_Typedef *simiic,uint8_t ack);
#endif

