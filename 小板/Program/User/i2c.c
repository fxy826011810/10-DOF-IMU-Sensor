#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "i2c.h" 
#include "tim.h" 
#include "config.h"
#include "delay.h"
#include "gpio.h" 
SimIIC_Typedef ist8310IIC;
SimIIC_Typedef ms5611IIC;

void Bsp_IIC_Init(void)
{
	#if USE_SIMIIC
	#else
	I2C_InitTypeDef i2c;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2CI|RCC_APB1Periph_I2CM,ENABLE);
	I2C_DeInit(I2CI);
	I2C_DeInit(I2CM);
	i2c.I2C_Ack=I2C_Ack_Disable;
	i2c.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	i2c.I2C_ClockSpeed=400000;
	i2c.I2C_DutyCycle=I2C_DutyCycle_2;
	i2c.I2C_Mode=I2C_Mode_I2C;
	i2c.I2C_OwnAddress1=0x0A;
	I2C_Init(I2CI,&i2c);
	i2c.I2C_OwnAddress1=0x0B;
	I2C_Init(I2CM,&i2c);
	I2C_Cmd(I2CI, ENABLE);
	I2C_Cmd(I2CM, ENABLE);
	#endif
	
	
}
static void IIC_SCL(SimIIC_Typedef *simiic,uint8_t x)
{
		switch(x)
	{
		case 1:
			GPIO_SetBits(simiic->gpioScl,simiic->scl_gpio_init.GPIO_Pin);
		break;
		case 0:
			GPIO_ResetBits(simiic->gpioScl,simiic->scl_gpio_init.GPIO_Pin);
		break;
	}
}
static void IIC_SDA(SimIIC_Typedef *simiic,uint8_t x)
{
		switch(x)
	{
		case 1:
			GPIO_SetBits(simiic->gpioSda,simiic->sda_gpio_init.GPIO_Pin);
		break;
		case 0:
			GPIO_ResetBits(simiic->gpioSda,simiic->sda_gpio_init.GPIO_Pin);
		break;
	}
}
static uint8_t READ_SDA(SimIIC_Typedef *simiic)
{
	return GPIO_ReadInputDataBit(simiic->gpioSda,simiic->sda_gpio_init.GPIO_Pin);
}
static void SDA_IN(SimIIC_Typedef *simiic)
{
	simiic->sda_gpio_init.GPIO_Mode							= GPIO_Mode_IN;
	simiic->sda_gpio_init.GPIO_PuPd							= GPIO_PuPd_NOPULL;
	GPIO_Init(simiic->gpioSda, &simiic->sda_gpio_init);
}
static void SDA_OUT(SimIIC_Typedef *simiic)
{
	simiic->sda_gpio_init.GPIO_Mode							= GPIO_Mode_OUT;
	simiic->sda_gpio_init.GPIO_OType							= GPIO_OType_PP;
	simiic->sda_gpio_init.GPIO_PuPd							= GPIO_PuPd_NOPULL;
	simiic->sda_gpio_init.GPIO_Speed							= GPIO_Speed_50MHz;
	GPIO_Init(simiic->gpioSda, &simiic->sda_gpio_init);
}
 
static uint8_t IIC_Start(SimIIC_Typedef *simiic)
{
	SDA_OUT(simiic);     //sda线输出
	IIC_SDA(simiic,1);	  	  
	IIC_SCL(simiic,1);
	delay_us(4);
  if(!READ_SDA(simiic))
  return 1;
 	IIC_SDA(simiic,0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL(simiic,0);//钳住I2C总线，准备发送或接收数据 
  return 0;
}	  
//产生IIC停止信号
static void IIC_Stop(SimIIC_Typedef *simiic)
{
	SDA_OUT(simiic);//sda线输出
	IIC_SCL(simiic,0);
	IIC_SDA(simiic,0);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL(simiic,1);
	IIC_SDA(simiic,1);//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
static uint8_t IIC_Wait_Ack(SimIIC_Typedef *simiic)
{
	u8 ucErrTime=0;
	SDA_IN(simiic);      //SDA设置为输入  
	IIC_SDA(simiic,1);delay_us(1);   
	IIC_SCL(simiic,1);delay_us(1);
	while(READ_SDA(simiic))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop(simiic);
			return 1;
		}
	}
	IIC_SCL(simiic,0);//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
static void IIC_NAck(SimIIC_Typedef *simiic)
{
	IIC_SCL(simiic,0);
	SDA_OUT(simiic);
	IIC_SDA(simiic,1);///////改
	delay_us(2);
	IIC_SCL(simiic,1);
	delay_us(2);
	IIC_SCL(simiic,0);
}
//不产生ACK应答		    
 void IIC_Ack(SimIIC_Typedef *simiic)
{
	IIC_SCL(simiic,0);
	SDA_OUT(simiic);
	IIC_SDA(simiic,0);///////改
	delay_us(2);
	IIC_SCL(simiic,1);
	delay_us(2);
	IIC_SCL(simiic,0);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
 static void IIC_Send_Byte(SimIIC_Typedef *simiic,uint8_t txd)
{                        
    uint8_t t;   
    SDA_OUT(simiic); 	    
    IIC_SCL(simiic,0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if(txd&0x80)
			IIC_SDA(simiic,1);
		else
			IIC_SDA(simiic,0);
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL(simiic,1);
		delay_us(2); 
		IIC_SCL(simiic,0);	
		delay_us(2);
    }
   // IIC_SCL(0);
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
static uint8_t IIC_Read_Byte(SimIIC_Typedef *simiic,uint8_t ack)
{
	unsigned char i,receive=0;
	SDA_IN(simiic);//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL(simiic,0); 
        delay_us(2);
				IIC_SCL(simiic,1);
        receive<<=1;
        delay_us(2);
        if(READ_SDA(simiic)) receive|=0x01;   
				delay_us(1); 
        }
    IIC_SCL(simiic,0);
//    delay_us(2);
    if (!ack)
        IIC_NAck(simiic);//发送nACK
    else
        IIC_Ack(simiic); //发送ACK   
    return receive;
}


uint8_t IIC_WriteByte(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data)
{
  IIC_Start(simiic);
  IIC_Send_Byte(simiic,simiic->Addr<<1);
  if(IIC_Wait_Ack(simiic))
  {
	  return 1;
  }
  IIC_Send_Byte(simiic,reg);
  if (IIC_Wait_Ack(simiic))
  {
	  return 1;
  }
	if(simiic->writedataflag==1)
	{
		IIC_Send_Byte(simiic,Data);
		if (IIC_Wait_Ack(simiic))
		{
			return 1;
		}
	}
  IIC_Stop(simiic);
  return 0;
}

uint8_t IIC_ReadByte(SimIIC_Typedef *simiic,uint8_t reg, uint8_t *pbuffer)
{				
 
    IIC_Start(simiic);
    IIC_Send_Byte(simiic,simiic->Addr << 1);   //发送器件地址0XA0,写数据 	 
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
    IIC_Send_Byte(simiic,reg);   //发送低地址
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
    IIC_Start(simiic);  	 	   
    IIC_Send_Byte(simiic,(simiic->Addr << 1)+1);           //进入接收模式			   
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
	*pbuffer =IIC_Read_Byte(simiic,0);
    IIC_Stop(simiic);//产生一个停止条件	
	
    return 0;
}

uint8_t IIC_Read(SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len)
{ 
	IIC_Start(simiic);
	IIC_Send_Byte(simiic,simiic->Addr << 1);   //发送器件地址0XA0,写数据 	 
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
	IIC_Send_Byte(simiic,reg);   //发送低地址
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
   IIC_Start(simiic);
   IIC_Send_Byte(simiic,(simiic->Addr << 1)+1); 
   if (IIC_Wait_Ack(simiic))
	 {
		return 1;
	 }
	while (len)
	{
		if(len==1)*pbuffer = IIC_Read_Byte(simiic,0);
		else	*pbuffer = IIC_Read_Byte(simiic,1);		         			   	
		pbuffer++;
		len--;
	}
	IIC_Stop(simiic);//产生一个停止条件	  
	return 0;
	
}
uint8_t IIC_Write(SimIIC_Typedef *simiic, uint8_t reg, uint8_t *Data, uint8_t len)
{
	IIC_Start(simiic);
	IIC_Send_Byte(simiic,simiic->Addr << 1);
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
	IIC_Send_Byte(simiic,reg);
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
	while (len)
	{
		IIC_Send_Byte(simiic,*Data);
		if (IIC_Wait_Ack(simiic))
		{
			return 1;
		}
		Data++;
		len--;
	}
	IIC_Stop(simiic);
	return 0;
}

void IIC_H_WriteByte(I2C_TypeDef* I2Cx,uint8_t addr,uint8_t reg,uint8_t Data)
{
	
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

  I2C_GenerateSTART(I2Cx, ENABLE);
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));


  I2C_Send7bitAddress(I2Cx, addr<<1, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2Cx, reg);
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_SendData(I2Cx, Data);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTOP(I2Cx, ENABLE);
}



void IIC_H_Read(I2C_TypeDef* I2Cx,uint8_t addr,uint8_t reg,uint8_t *pbuffer, uint8_t len)
{ 
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	
	I2C_GenerateSTART(I2Cx, ENABLE);
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));


  I2C_Send7bitAddress(I2Cx, addr<<1, I2C_Direction_Transmitter);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	

  I2C_SendData(I2Cx, reg);
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	I2C_GenerateSTART(I2Cx, ENABLE);
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2Cx, addr<<1|0x01, I2C_Direction_Receiver);
  while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while (len)
	{
		
		if(len==1)
		{
			I2C_AcknowledgeConfig(I2Cx, DISABLE);
			while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
			*pbuffer =I2C_ReceiveData(I2Cx);
		}
		else
		{
			I2C_AcknowledgeConfig(I2Cx, ENABLE);
			while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
			*pbuffer =I2C_ReceiveData(I2Cx);
		}	         			   	
		pbuffer++;
		len--;
	}
  I2C_GenerateSTOP(I2Cx, ENABLE);
}





