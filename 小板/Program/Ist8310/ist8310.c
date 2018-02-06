#include "ist8310Dri.h"
#include "stm32f4xx.h"
#include "calibration.h"
#include "ist8310.h"
#include "config.h"
#include "delay.h"
#include "main.h"
#include "i2c.h" 
#include <string.h>

#define OTPsensitivity (165)

void Ist8310_Crossaxis_Matrix(float crossaxis_inv[9],int enable)
{
	uint8_t crossxbuf[6],crossybuf[6],crosszbuf[6];
	int16_t OTPcrossaxis[9] = {0};
	
	float inv[3][3] = {0},mat_A[3][3];

	IST8310_Read(IST8310_REG_XX_CROSS_L,crossxbuf,6);
	IST8310_Read(IST8310_REG_YX_CROSS_L,crossybuf,6);
	IST8310_Read(IST8310_REG_ZX_CROSS_L,crosszbuf,6);
	
	
	OTPcrossaxis[0] = ((int16_t) crossxbuf[1]) << 8 | crossxbuf[0];
  OTPcrossaxis[3] = ((int16_t) crossxbuf[3]) << 8 | crossxbuf[2];
  OTPcrossaxis[6] = ((int16_t) crossxbuf[5]) << 8 | crossxbuf[4];
  OTPcrossaxis[1] = ((int16_t) crossybuf[1]) << 8 | crossybuf[0];
  OTPcrossaxis[4] = ((int16_t) crossybuf[3]) << 8 | crossybuf[2];
  OTPcrossaxis[7] = ((int16_t) crossybuf[5]) << 8 | crossybuf[4];
  OTPcrossaxis[2] = ((int16_t) crosszbuf[1]) << 8 | crosszbuf[0];
  OTPcrossaxis[5] = ((int16_t) crosszbuf[3]) << 8 | crosszbuf[2];
  OTPcrossaxis[8] = ((int16_t) crosszbuf[5]) << 8 | crosszbuf[4];
	
	mat_A[0][0]=(float)OTPcrossaxis[0]/2;
  mat_A[1][0]=(float)OTPcrossaxis[3]/2;
  mat_A[2][0]=(float)OTPcrossaxis[6]/2;
  mat_A[0][1]=(float)OTPcrossaxis[1]/2;
  mat_A[1][1]=(float)OTPcrossaxis[4]/2;
  mat_A[2][1]=(float)OTPcrossaxis[7]/2;
  mat_A[0][2]=(float)OTPcrossaxis[2]/2;
  mat_A[1][2]=(float)OTPcrossaxis[5]/2;
  mat_A[2][2]=(float)OTPcrossaxis[8]/2;
	
	mat_invert3(mat_A,inv);
      
  for (int i=0; i<3; i++) 
	{
		for (int j=0; j<3; j++) 
		{
			crossaxis_inv[j+3*i] = inv[i][j] * OTPsensitivity;
		}
	}
}





void Ist8310_CrossaxisTransformation(float crossaxis_inv[9],magDatadef *m,magDatadef *o)
{
	int i = 0;
	float outputtmp[3];
	for( i = 0; i < 9; i++)
	{
        if(crossaxis_inv[i]!=0)
          break;
        if(i == 8)
				{
					IST8310_SetStatus(0);
          Ist8310_Init();
				}
  }

  outputtmp[0] = m->mx * crossaxis_inv[0] +
                 m->my * crossaxis_inv[1] +
                 m->mz * crossaxis_inv[2];

  outputtmp[1] = m->mx * crossaxis_inv[3] +
                 m->my * crossaxis_inv[4] +
                 m->mz * crossaxis_inv[5];
    
  outputtmp[2] = m->mx * crossaxis_inv[6] +
                 m->my * crossaxis_inv[7] +
                 m->mz * crossaxis_inv[8];
	o->mx= (short)(outputtmp[0])-16;
  o->my= (short)(outputtmp[1])-300;
  o->mz= (short)(outputtmp[2])-201;
}
	
	


void Ist8310_Init(void)
{
#if USE_SIMIIC
	ist8310IIC.writedataflag=1;
	ist8310IIC.Addr=IST8310_ADDR;
	ist8310IIC.ReadByte=&IIC_ReadByte;
	ist8310IIC.ReadBytes=&IIC_Read;
	ist8310IIC.WriteByte=&IIC_WriteByte;
	ist8310IIC.WriteBytes=&IIC_Write;
#endif	
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
	if(0x24==c&&0xc0==d)
	{
		IST8310_SetStatus(1);
	}
	
	
	int crossaxis_enable = 0;
	char cross_mask=0xFF;
  uint8_t wbuffer[2];

	IST8310_Read(IST8310_REG_XX_CROSS_L,wbuffer,2);
	if((wbuffer[0] == cross_mask) && (wbuffer[1] == cross_mask))
        crossaxis_enable = 0;
    else
        crossaxis_enable = 1;
	Ist8310_Crossaxis_Matrix(cmd.Ist8310.Crossaxis,crossaxis_enable);
	
}

void Ist8310_DataUpdate(void)
{
	IST8310_GetData(&cmd.Ist8310.Data.original);
	Ist8310_CrossaxisTransformation(cmd.Ist8310.Crossaxis,&cmd.Ist8310.Data.original,&cmd.Ist8310.Data.calc);
//	memcpy(&cmd.Ist8310.Data.calc,&cmd.Ist8310.Data.original,sizeof(cmd.Ist8310.Data.original));		
	IST8310_SetDataStatus(1);
	Monitor_Set(&cmd.Ist8310.monitor);
}


void IST8310_GetMagData(uint8_t *data)
{

	IST8310_Read(IST8310_R_XL,data,6);
	
	IST8310_WriteByte(IST8310_R_CONFA,IST8310_ODR_MODE);
}
void IST8310_GetData(magDatadef *m)
{
	uint8_t data[6];
	IST8310_GetMagData(data);
	m->mx=data[1]<<8|data[0];
	m->my=data[3]<<8|data[2];
	m->mz=data[5]<<8|data[4];
}
uint8_t IST8310_GetStatus(void)
{
	return cmd.Ist8310.status;
}
void IST8310_SetStatus(uint8_t x)
{
	cmd.Ist8310.status=x;
}
uint8_t IST8310_GetDataStatus(void)
{
	return cmd.Ist8310.dataStatus;
}
void IST8310_SetDataStatus(uint8_t x)
{
	cmd.Ist8310.dataStatus=x;
}

uint8_t IST8310_GetIntStatus(uint8_t flag)
{
	uint8_t a;
	if(flag)
	{
	return GPIO_ReadInputDataBit(IST8310_INT_GPIO,IST8310_INT_PIN);
	}
	else
	{
	 IST8310_ReadByte(IST8310_R_MODE,&a);
	return a&0x01;
	}
//	return 0;
}







