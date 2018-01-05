#include "stm32f4xx.h"
#include "ms5611.h"
#include "delay.h"
#include "config.h"
#include "i2c.h" 
#include <stdio.h>
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

uint8_t Ms5611_Reset(void)
{
	return ms5611IIC.WriteByte(&ms5611IIC,RESET,NULL);
}

uint16_t calcbuffer[8];
uint8_t Ms5611_PromRead(SimIIC_Typedef *simiic,uint8_t reg,uint8_t Data)
{
uint8_t i,buffer[16];
	for(i=0;i<8;i++)
	{
//		Ms5611_WriteByte(&ms5611IIC,PROM_READ+2*i,0);
//		Ms5611_ReadBytes(&ms5611IIC,&buffer[2*i],2);
		ms5611IIC.ReadBytes(&ms5611IIC,PROM_READ+2*i,&buffer[2*i],2);
		calcbuffer[i]=buffer[2*i]<<8|buffer[2*i+1];
		delay_ms(10);
	}
	return 0;
}

void Ms5611_Init(void)
{
	ms5611IIC.writedataflag=0;
	ms5611IIC.Addr=MS5611_ADDR;
	ms5611IIC.ReadByte=&IIC_ReadByte;
	ms5611IIC.ReadBytes=&IIC_Read;
	ms5611IIC.WriteByte=&IIC_WriteByte;
	ms5611IIC.WriteBytes=&IIC_Write;
	
	
	Ms5611_Reset();
	delay_ms(100);
	Ms5611_PromRead(&ms5611IIC,NULL,NULL);

}

//void Ms5611_ReadByte(uint8_t reg, uint8_t *pbuffer)
//{
////	IIC_ReadByte(&ms5611IIC,reg,pbuffer);
//}

//void Ms5611_Read(uint8_t reg,  uint8_t *pbuffer, uint8_t len)
//{
//	ms5611IIC.ReadBytes(&ms5611IIC,reg,pbuffer,len);
//}

//void Ms5611_WriteByte(uint8_t reg, uint8_t pbuffer)
//{
//	ms5611IIC.WriteByte(&ms5611IIC,reg,pbuffer);
//}

uint8_t Ms5611_ReadD(uint8_t reg,uint32_t *trans)
{
	uint8_t data[3];
	Ms5611_WriteByte(&ms5611IIC,reg,1);
	delay_ms(10);
	Ms5611_WriteByte(&ms5611IIC,ADC_READ,0);
	Ms5611_ReadBytes(&ms5611IIC,data,3);
	*trans=data[0]<<16|data[1]<<8|data[2];
  return 0;
}








