#include "main.h"
void Bsp_IIC_Init(void)
{
	
}
 uint8_t IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay_us(4);
  if(!READ_SDA)
  return 1;
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
        return 0;
}	  
//����IICֹͣ�ź�
 void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL(0);
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL(1);
	IIC_SDA(1);//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
 uint8_t IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA(1);	   
	IIC_SCL(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
 void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);///////��
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}
//������ACKӦ��		    
 void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);///////��
	delay_us(2);
	IIC_SCL(1);
	delay_us(2);
	IIC_SCL(0);
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
 void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
    SDA_OUT(); 	    
    IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if(txd&0x80)
			IIC_SDA(1);
		else
			IIC_SDA(0);
		txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL(1);
		delay_us(2); 
		IIC_SCL(0);	
		delay_us(2);
    }
   // IIC_SCL(0);
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
 uint8_t IIC_Read_Byte(uint8_t ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL(0); 
        delay_us(2);
				IIC_SCL(1);
        receive<<=1;
        delay_us(2);
        if(READ_SDA) receive|=0x01;   
				delay_us(1); 
        }
    IIC_SCL(0);
    delay_us(2);
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}


uint8_t IIC_WriteByte(uint8_t Addr,uint8_t reg,uint8_t Data)
{
  IIC_Start();
  IIC_Send_Byte(Addr<<1);
  if(IIC_Wait_Ack())
  {
	  printf("Addr error");
	  return 1;
  }
  IIC_Send_Byte(reg);
  if (IIC_Wait_Ack())
  {
	  printf("reg error");
	  return 1;
  }
  IIC_Send_Byte(Data);
  if (IIC_Wait_Ack())
  {
	  printf("Data error");
	  return 1;
  }
  IIC_Stop();
  return 0;
  //delay_ms(10);
}

uint8_t IIC_ReadByte(uint8_t Addr,uint8_t reg, uint8_t *pbuffer)
{				  
    IIC_Start();
    IIC_Send_Byte(Addr << 1);   //����������ַ0XA0,д���� 	 
	if (IIC_Wait_Ack())
	{
		printf("WAddr error");
		return 1;
	}
    IIC_Send_Byte(reg);   //���͵͵�ַ
	if (IIC_Wait_Ack())
	{
		printf("reg error");
		return 1;
	}
    IIC_Start();  	 	   
    IIC_Send_Byte((Addr << 1)+1);           //�������ģʽ			   
	if (IIC_Wait_Ack())
	{
		printf("RAddr error");
		return 1;
	}
	*pbuffer =IIC_Read_Byte(0);
    IIC_Stop();//����һ��ֹͣ����	    
    return 0;
}

uint8_t IIC_Read_x(uint8_t Addr,uint8_t reg,uint8_t *pbuffer,uint8_t len)
{
	  while(len)
  {	  	    																 
IIC_ReadByte(Addr,reg,pbuffer);
pbuffer++;
reg++;
len--;

  }
	  return 0;
}
uint8_t IIC_Write_x(uint8_t Addr,uint8_t reg,uint8_t *Data,uint8_t len)
{
  while(len)
  {
  IIC_WriteByte( Addr, reg, *Data);
  Data++;
  reg++;
  len--;
  }
  return 0;
}
uint8_t IIC_Read1(uint8_t Addr,uint8_t reg,uint8_t len,uint8_t *pbuffer)
{
	  while(len)
  {	  	    																 
IIC_ReadByte(Addr,reg,pbuffer);
pbuffer++;
reg++;
len--;

  }
	  return 0;
}
uint8_t IIC_Write1(uint8_t Addr,uint8_t reg,uint8_t len,uint8_t *Data)
{
  while(len)
  {
  IIC_WriteByte( Addr, reg, *Data);
  Data++;
  reg++;
  len--;
  }
  return 0;
}
uint8_t IIC_Read(uint8_t Addr, uint8_t reg,uint8_t *pbuffer, uint8_t len)
{

	IIC_Start();
	IIC_Send_Byte(Addr << 1);   //����������ַ0XA0,д���� 	 
	if (IIC_Wait_Ack())
	{
		printf("WAddr error");
		return 1;
	}
	IIC_Send_Byte(reg);   //���͵͵�ַ
	if (IIC_Wait_Ack())
	{
		printf("reg error");
		return 1;
	}
        IIC_Start();
        IIC_Send_Byte((Addr << 1)|1); 
        if (IIC_Wait_Ack())
		{
			printf("RAddr error");
			return 1;
		}
	while (len)
	{
	if(len==1)*pbuffer = IIC_Read_Byte(0);
        else	*pbuffer = IIC_Read_Byte(1);		         			   	
        pbuffer++;
        len--;
	}
	IIC_Stop();//����һ��ֹͣ����	    
	return 0;
}
uint8_t IIC_Write(uint8_t Addr, uint8_t reg, uint8_t *Data, uint8_t len)
{
	IIC_Start();
	IIC_Send_Byte(Addr << 1);
	if (IIC_Wait_Ack())
	{
		printf("Addr error");
		return 1;
	}
	IIC_Send_Byte(reg);
	if (IIC_Wait_Ack())
	{
		printf("reg error");
		return 1;
	}
	while (len)
	{
		IIC_Send_Byte(*Data);
		if (IIC_Wait_Ack())
		{
			printf("Data error");
			return 1;
		}
		Data++;
		len--;
	}

	IIC_Stop();
	return 0;
}


