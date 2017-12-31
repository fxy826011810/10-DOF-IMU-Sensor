#include "main.h"

static uint8_t IIC_WriteByte(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data);//д�����ֽ�
static uint8_t IIC_ReadByte(SimIIC_Typedef *simiic,uint8_t reg, uint8_t *pbuffer);//�������ֽ�
static uint8_t IIC_Read(SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len);//������ֽ�
static uint8_t IIC_Write(SimIIC_Typedef *simiic, uint8_t reg, uint8_t *Data, uint8_t len);//д����ֽ�


void Ist8310_Init(void)
{
	ist8310IIC.Addr=IST8310_ADDR;
	ist8310IIC.ReadByte=&IIC_ReadByte;
	ist8310IIC.ReadBytes=&IIC_Read;
	ist8310IIC.WriteByte=&IIC_WriteByte;
	ist8310IIC.WriteBytes=&IIC_Write;
	
	uint8_t id1=0,c,d;
	IST8310_ReadByte(IST8310_WHO_AM_I,&id1);
	delay_ms(10);
	IST8310_WriteByte(IST8310_AVGCNTL, 0x24);
	delay_ms(10);
	IST8310_ReadByte(IST8310_AVGCNTL,&c);
	delay_ms(10);
	IST8310_WriteByte(IST8310_PDCNTL, 0xc0);
	delay_ms(10);
	IST8310_ReadByte(IST8310_PDCNTL,&d);
	delay_ms(10);
	IST8310_WriteByte(IST8310_R_CONFA,IST8310_ODR_MODE);
	delay_ms(10);
	
}
void IST8310_GetMagData(uint8_t *data)
{
	IST8310_Read(IST8310_R_XL,data,6);
	IST8310_WriteByte(IST8310_R_CONFA,IST8310_ODR_MODE);
}
magDatedef mag;
void IST8310_GetData(magDatedef *m)
{
	uint8_t data[6];
	IST8310_GetMagData(data);
	m->mx=data[1]<<8|data[0];
	m->my=data[3]<<8|data[2];
	m->mz=data[5]<<8|data[4];
}

void IST8310_ReadByte(uint8_t reg, uint8_t *pbuffer)
{
	ist8310IIC.ReadByte(&ist8310IIC,reg,pbuffer);
}

void IST8310_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len)
{
	ist8310IIC.ReadBytes(&ist8310IIC,reg,pbuffer,len);
}
void IST8310_WriteByte(uint8_t reg, uint8_t pbuffer)
{
	ist8310IIC.WriteByte(&ist8310IIC,reg,pbuffer);
}
static uint8_t IIC_WriteByte(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data)
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
  IIC_Send_Byte(simiic,Data);
  if (IIC_Wait_Ack(simiic))
  {
	  return 1;
  }
  IIC_Stop(simiic);
  return 0;
}

static uint8_t IIC_ReadByte(SimIIC_Typedef *simiic,uint8_t reg, uint8_t *pbuffer)
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
static uint8_t IIC_Read(SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len)
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
        IIC_Send_Byte(simiic,(simiic->Addr << 1)|1); 
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
static uint8_t IIC_Write(SimIIC_Typedef *simiic, uint8_t reg, uint8_t *Data, uint8_t len)
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
