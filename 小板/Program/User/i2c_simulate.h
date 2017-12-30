#ifndef __I2C_SIMULATE_H
#define __I2C_SIMULATE_H



#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式


//IO操作函数	 
//a &= ~bit
//GPIOB->ODR=(GPIOB->ODR&=~GPIO_Pin_6)|(x <<6)
#define IIC_SCL(x)    GPIOB->ODR=((GPIOB->ODR&(~GPIO_Pin_8))|(x <<8)) //SCL
#define IIC_SDA(x)    GPIOB->ODR=((GPIOB->ODR&(~GPIO_Pin_9))|(x <<9)) //SDA	 
#define READ_SDA      GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)//输入SDA 
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

