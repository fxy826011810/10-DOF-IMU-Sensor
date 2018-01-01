#include "main.h"
static uint8_t IIC_Read(SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len);
static uint8_t IIC_Write(SimIIC_Typedef *simiic, uint8_t reg, uint8_t *Data, uint8_t len);
static uint8_t IIC_ReadByte(SimIIC_Typedef *simiic,uint8_t reg, uint8_t *pbuffer);
static uint8_t IIC_WriteByte(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data);
void Ms5611_Init(void)
{
	ms5611IIC.ReadByte=&IIC_ReadByte;
	ms5611IIC.ReadBytes=&IIC_Read;
	ms5611IIC.WriteByte=&IIC_WriteByte;
	ms5611IIC.WriteBytes=&IIC_Write;
	ms5611IIC.Addr=MS5611_ADDR;
	Ms5611_Reset();
	while(1)
	{
	delay_ms(100);
	Ms5611_PromRead(&ms5611IIC,NULL,NULL);
	}
}





//void Ms5611_ReadByte(uint8_t reg, uint8_t *pbuffer)
//{
////	IIC_ReadByte(&ms5611IIC,reg,pbuffer);
//}

void Ms5611_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len)
{
	ms5611IIC.ReadBytes(&ms5611IIC,reg,pbuffer,len);
}

//void Ms5611_WriteByte(uint8_t reg, uint8_t pbuffer)
//{
//	ms5611IIC.WriteByte(&ms5611IIC,reg,pbuffer);
//}

uint8_t Ms5611_StartADC(void)
{
	ms5611IIC.WriteByte(&ms5611IIC,Convert_D2_4096,1);
  return 0;
}




uint8_t Ms5611data[3]={0};
uint8_t Ms5611_ReadADC(void)
{
	Ms5611_WriteByte(&ms5611IIC,Convert_D1_256,1);
	delay_ms(10);
	Ms5611_WriteByte(&ms5611IIC,ADC_READ,1);
	//delay_us(600);
	
	Ms5611_ReadBytes(&ms5611IIC,Ms5611data,3);
  return 0;
}
uint8_t Ms5611_Reset(void)
{
	return Ms5611_WriteByte(&ms5611IIC,RESET,1);
}

uint16_t calcbuffer[8];
uint8_t Ms5611_PromRead(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data)
{
uint8_t i,buffer[16];
	for(i=0;i<=8;i++)
	{
		Ms5611_WriteByte(&ms5611IIC,PROM_READ+2*i,0);
		Ms5611_ReadBytes(&ms5611IIC,&buffer[i],2);
		calcbuffer[i]=buffer[i]<<8|buffer[i+1];
	}
	return 0;
}



uint8_t Ms5611_WriteByte(SimIIC_Typedef *simiic,uint8_t reg,uint8_t flag)
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
	if(flag)
  IIC_Stop(simiic);
  return 0;
}

uint8_t Ms5611_ReadBytes(SimIIC_Typedef *simiic,uint8_t *pbuffer,uint8_t len)
{
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
static uint8_t IIC_Read(SimIIC_Typedef *simiic, uint8_t reg,uint8_t *pbuffer, uint8_t len)
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
	IIC_Stop(simiic);
	delay_ms(10);
        IIC_Start(simiic);
        IIC_Send_Byte(simiic,(simiic->Addr << 1)|0x01); 
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


