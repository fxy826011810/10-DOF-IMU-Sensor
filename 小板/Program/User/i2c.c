#include "stm32f4xx.h"
#include "i2c.h" 
#include "config.h"
#include "delay.h"

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
			GPIO_SetBits(simiic->gpioScl,simiic->scl_gpio_init.GPIO_Pin);
		break;
		case 0:
			GPIO_ResetBits(simiic->gpioScl,simiic->scl_gpio_init.GPIO_Pin);
		break;
	}
}
void IIC_SDA(SimIIC_Typedef *simiic,uint8_t x)
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
uint8_t READ_SDA(SimIIC_Typedef *simiic)
{
	return GPIO_ReadInputDataBit(simiic->gpioSda,simiic->sda_gpio_init.GPIO_Pin);
}
void SDA_IN(SimIIC_Typedef *simiic)
{
	simiic->sda_gpio_init.GPIO_Mode							= GPIO_Mode_IN;
	simiic->sda_gpio_init.GPIO_PuPd							= GPIO_PuPd_NOPULL;
	GPIO_Init(simiic->gpioSda, &simiic->sda_gpio_init);
}
void SDA_OUT(SimIIC_Typedef *simiic)
{
	simiic->sda_gpio_init.GPIO_Mode							= GPIO_Mode_OUT;
	simiic->sda_gpio_init.GPIO_OType							= GPIO_OType_PP;
	simiic->sda_gpio_init.GPIO_PuPd							= GPIO_PuPd_NOPULL;
	simiic->sda_gpio_init.GPIO_Speed							= GPIO_Speed_50MHz;
	GPIO_Init(simiic->gpioSda, &simiic->sda_gpio_init);
}
 
uint8_t IIC_Start(SimIIC_Typedef *simiic)
{
	SDA_OUT(simiic);     //sda�����
	IIC_SDA(simiic,1);	  	  
	IIC_SCL(simiic,1);
	delay_us(4);
  if(!READ_SDA(simiic))
  return 1;
 	IIC_SDA(simiic,0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL(simiic,0);//ǯסI2C���ߣ�׼�����ͻ�������� 
  return 0;
}	  
//����IICֹͣ�ź�
void IIC_Stop(SimIIC_Typedef *simiic)
{
	SDA_OUT(simiic);//sda�����
	IIC_SCL(simiic,0);
	IIC_SDA(simiic,0);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL(simiic,1);
	IIC_SDA(simiic,1);//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
 uint8_t IIC_Wait_Ack(SimIIC_Typedef *simiic)
{
	u8 ucErrTime=0;
	SDA_IN(simiic);      //SDA����Ϊ����  
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
	IIC_SCL(simiic,0);//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
 void IIC_NAck(SimIIC_Typedef *simiic)
{
	IIC_SCL(simiic,0);
	SDA_OUT(simiic);
	IIC_SDA(simiic,1);///////��
	delay_us(2);
	IIC_SCL(simiic,1);
	delay_us(2);
	IIC_SCL(simiic,0);
}
//������ACKӦ��		    
 void IIC_Ack(SimIIC_Typedef *simiic)
{
	IIC_SCL(simiic,0);
	SDA_OUT(simiic);
	IIC_SDA(simiic,0);///////��
	delay_us(2);
	IIC_SCL(simiic,1);
	delay_us(2);
	IIC_SCL(simiic,0);
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
 void IIC_Send_Byte(SimIIC_Typedef *simiic,uint8_t txd)
{                        
    uint8_t t;   
    SDA_OUT(simiic); 	    
    IIC_SCL(simiic,0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if(txd&0x80)
			IIC_SDA(simiic,1);
		else
			IIC_SDA(simiic,0);
		txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL(simiic,1);
		delay_us(2); 
		IIC_SCL(simiic,0);	
		delay_us(2);
    }
   // IIC_SCL(0);
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(SimIIC_Typedef *simiic,uint8_t ack)
{
	unsigned char i,receive=0;
	SDA_IN(simiic);//SDA����Ϊ����
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
        IIC_NAck(simiic);//����nACK
    else
        IIC_Ack(simiic); //����ACK   
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
    IIC_Send_Byte(simiic,simiic->Addr << 1);   //����������ַ0XA0,д���� 	 
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
    IIC_Send_Byte(simiic,reg);   //���͵͵�ַ
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
    IIC_Start(simiic);  	 	   
    IIC_Send_Byte(simiic,(simiic->Addr << 1)+1);           //�������ģʽ			   
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
	*pbuffer =IIC_Read_Byte(simiic,0);
    IIC_Stop(simiic);//����һ��ֹͣ����	    
    return 0;
}
uint8_t IIC_Read(SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len)
{

	IIC_Start(simiic);
	IIC_Send_Byte(simiic,simiic->Addr << 1);   //����������ַ0XA0,д���� 	 
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
	IIC_Send_Byte(simiic,reg);   //���͵͵�ַ
	if (IIC_Wait_Ack(simiic))
	{
		return 1;
	}
//	if(simiic->writedataflag==0)
//	{
//		IIC_Stop(simiic);
//	}
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
	IIC_Stop(simiic);//����һ��ֹͣ����	    
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







