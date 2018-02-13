#include "i2c.h" 
#include "ms5611.h" 
#include "ms5611Dri.h" 
#include "delay.h" 
#include "stdio.h" 
#include "main.h" 
#include "stm32f4xx.h" 
#include "monitor.h" 
Ms5611_t ms5611;
static uint8_t Ms5611_PromRead(uint8_t reg,uint16_t *Data)
{
uint8_t i,buffer[16];
	for(i=0;i<8;i++)
	{
		Ms5611_Read(PROM_READ+2*i,&buffer[2*i],2);
		Data[i]=buffer[2*i]<<8|buffer[2*i+1];
		delay_ms(10);
	}
	return 0;
}

void Ms5611_Reset(void)
{
	 Ms5611_WriteByte(RESET,NULL);
}

void Ms5611_Init(void)
{
	
	ms5611.Status=prepareTempADC;
#if USE_SIMIIC
	ms5611IIC.writedataflag=0;
	ms5611IIC.Addr=MS5611_ADDR;
	ms5611IIC.ReadByte=&IIC_ReadByte;
	ms5611IIC.ReadBytes=&IIC_Read;
	ms5611IIC.WriteByte=&IIC_WriteByte;
	ms5611IIC.WriteBytes=&IIC_Write;
#endif	
	Ms5611_Reset();
	delay_ms(100);
	Ms5611_PromRead(NULL,ms5611.Data.prom);
}


uint8_t Ms5611_ReadD(Ms5611_t *ms)
{
	uint8_t ms5611Registerdata[3];
	 if(ms->Status==readTempADC)
	{
		Ms5611_Read(ADC_READ,ms5611Registerdata,3);
		ms->Data.temp=ms5611Registerdata[0]<<16|ms5611Registerdata[1]<<8|ms5611Registerdata[2];
		ms->Status=preparePressureADC;
	}
	else if(ms->Status==readPressureADC)
	{
		Ms5611_Read(ADC_READ,ms5611Registerdata,3);
		ms->Data.pressure=ms5611Registerdata[0]<<16|ms5611Registerdata[1]<<8|ms5611Registerdata[2];
		ms->Status=prepareTempADC;
	}
	
	if(ms->Status==prepareTempADC)
	{
		Ms5611_WriteByte(Convert_D2_2048,NULL);
		ms->Status=readTempADC;
	}
	else if(ms->Status==preparePressureADC)
	{
		Ms5611_WriteByte(Convert_D1_2048,NULL);
		ms->Status=readPressureADC;
	}
  return 0;
}


void Ms5611_DataUpdate(void)
{
	Ms5611_ReadD(&ms5611);
}





