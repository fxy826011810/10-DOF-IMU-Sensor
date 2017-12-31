#include "main.h"
SimIIC_Typedef ist8310IIC;
SimIIC_Typedef ms5611IIC;

void Bsp_IIC_Init(void)
{
	
	
	
}
void IIC_SCL(SimIIC_Typedef *simiic,uint8_t x)
{
		switch(x)
	{
		case 1:
			GPIO_SetBits(simiic->gpio,simiic->scl_gpio_init.GPIO_Pin);
		break;
		case 0:
			GPIO_ResetBits(simiic->gpio,simiic->scl_gpio_init.GPIO_Pin);
		break;
	}
}
void IIC_SDA(SimIIC_Typedef *simiic,uint8_t x)
{
		switch(x)
	{
		case 1:
			GPIO_SetBits(simiic->gpio,simiic->sda_gpio_init.GPIO_Pin);
		break;
		case 0:
			GPIO_ResetBits(simiic->gpio,simiic->sda_gpio_init.GPIO_Pin);
		break;
	}
}
uint8_t READ_SDA(SimIIC_Typedef *simiic)
{
	return GPIO_ReadInputDataBit(simiic->gpio,simiic->sda_gpio_init.GPIO_Pin);
}
void SDA_IN(SimIIC_Typedef *simiic)
{
	simiic->sda_gpio_init.GPIO_Mode							= GPIO_Mode_IN;
	simiic->sda_gpio_init.GPIO_PuPd							= GPIO_PuPd_NOPULL;
	GPIO_Init(simiic->gpio, &simiic->sda_gpio_init);
}
void SDA_OUT(SimIIC_Typedef *simiic)
{
	simiic->sda_gpio_init.GPIO_Mode							= GPIO_Mode_OUT;
	simiic->sda_gpio_init.GPIO_OType							= GPIO_OType_PP;
	simiic->sda_gpio_init.GPIO_Pin								= I2CI_SDA_PIN;
	simiic->sda_gpio_init.GPIO_PuPd							= GPIO_PuPd_NOPULL;
	simiic->sda_gpio_init.GPIO_Speed							= GPIO_Speed_50MHz;
	GPIO_Init(simiic->gpio, &simiic->sda_gpio_init);
}
 
uint8_t IIC_Start(SimIIC_Typedef *simiic)
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
void IIC_Stop(SimIIC_Typedef *simiic)
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
 uint8_t IIC_Wait_Ack(SimIIC_Typedef *simiic)
{
	u8 ucErrTime=0;
	SDA_IN(simiic);      //SDA设置为输入  
	IIC_SDA(simiic,1);	   
	IIC_SCL(simiic,1);	 
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
 void IIC_NAck(SimIIC_Typedef *simiic)
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
 void IIC_Send_Byte(SimIIC_Typedef *simiic,uint8_t txd)
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
uint8_t IIC_Read_Byte(SimIIC_Typedef *simiic,uint8_t ack)
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
    delay_us(2);
    if (!ack)
        IIC_NAck(simiic);//发送nACK
    else
        IIC_Ack(simiic); //发送ACK   
    return receive;
}










