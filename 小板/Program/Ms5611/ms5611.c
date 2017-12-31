#include "main.h"

void Ms5611_Init(void)
{
	ms5611IIC.Addr=MS5611_ADDR;
}
void Ms5611_ReadByte(uint8_t reg, uint8_t *pbuffer)
{
//	IIC_ReadByte(&ms5611IIC,reg,pbuffer);
}

void Ms5611_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len)
{
//	IIC_Read(&ms5611IIC,reg,pbuffer,len);
}
void Ms5611_WriteByte(uint8_t reg, uint8_t pbuffer)
{
//	IIC_WriteByte(&ms5611IIC,reg,pbuffer);
}
void Ms5611_GetData(magDatedef *m)
{

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