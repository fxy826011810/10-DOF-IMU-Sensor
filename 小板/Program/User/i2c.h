#ifndef __I2C_H
#define __I2C_H
#include "stm32f4xx.h"

typedef struct SimIIC_Typedef
{
	uint8_t Addr;
	uint8_t writedataflag;
	GPIO_TypeDef *gpioSda;
	GPIO_TypeDef *gpioScl;
	GPIO_InitTypeDef	sda_gpio_init;
	GPIO_InitTypeDef	scl_gpio_init;
	uint8_t (*WriteByte)(struct SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data);
	uint8_t (*WriteBytes)(struct SimIIC_Typedef *simiic, uint8_t reg, uint8_t *Data, uint8_t len);
	uint8_t (*ReadByte)(struct SimIIC_Typedef *simiic,uint8_t reg, uint8_t *pbuffer);
	uint8_t (*ReadBytes)(struct SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len);
}SimIIC_Typedef;

extern SimIIC_Typedef ist8310IIC;
extern SimIIC_Typedef ms5611IIC;

void Bsp_IIC_Init(void);

uint8_t IIC_WriteByte(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data);//写单个字节
uint8_t IIC_ReadByte(SimIIC_Typedef *simiic,uint8_t reg, uint8_t *pbuffer);//读单个字节
uint8_t IIC_Read(SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len);//读多个字节
uint8_t IIC_Write(SimIIC_Typedef *simiic, uint8_t reg, uint8_t *Data, uint8_t len);//写多个字节

void IIC_H_Read(I2C_TypeDef* I2Cx,uint8_t addr,uint8_t reg,uint8_t *pbuffer, uint8_t len);
void IIC_H_WriteByte(I2C_TypeDef* I2Cx,uint8_t addr,uint8_t reg,uint8_t Data);

#endif

