#include "stm32f4xx.h"
#include "icm20602.h"
#include "icm20602Dri.h"
#include "calibration.h"
#include "config.h"
#include "delay.h"
#include "main.h"

	void Icm20602_calculate_calibration_values(float accel_ref[6][3], float *accel_T, float *accel_offs, float g)
{
	float mat_A[3][3],
				mat_A_inv[3][3],
				a=0;
	
	/* calculate offsets */
	for (unsigned i = 0; i < 3; i++) {
		accel_offs[i] = (accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i]) / 2;
	}

	/* fill matrix A for linear equations system*/
	
	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
		  a = accel_ref[i * 2][j] - accel_offs[j];
			mat_A[i][j] = a;
		}
	}

	/* calculate inverse matrix for A */
	mat_invert3(mat_A, mat_A_inv);


	/* copy results to accel_T */
	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			/* simplify matrices mult because b has only one non-zero element == g at index i */
			accel_T[j+i*3] = mat_A_inv[i][j] * g;
		}
	}
}
	
	
	
	
	
	
uint8_t Icm20602_init(void)
{
	
	uint8_t len=12,mpu6500_id=0;
	uint8_t i,initdata[][2]={ {ICM20602_PWR_MGMT_1,0x80},\
														{ICM20602_PWR_MGMT_1,0x01},\
														{ICM20602_INT_PIN_CFG,0x00},\
														{ICM20602_INT_ENABLE,0x01},\
														{ICM20602_PWR_MGMT_2,0x00},\
														{ICM20602_SMPLRT_DIV,0x00},\
														{ICM20602_GYRO_CONFIG,0x18},\
														{ICM20602_ACCEL_CONFIG,0x08},\
														{ICM20602_ACCEL_CONFIG_2,0x03},\
														{ICM20602_USER_CTRL,0x01},\
														{ICM20602_CONFIG,0x03},\
														{ICM20602_I2C_IF,0x40},\
													};
	

	Icm20602_SetStatus(PinInt);
	Icm20602_SetDataStatus(0);
	Icm20602_Set_AccelCalibration_Status(0);
//	Icm20602_Set_Calibration_Status(1);
	Icm20602_ReadByte(ICM20602_WHO_AM_I,&mpu6500_id);								
	if (mpu6500_id != ICM20602_WHO_AM_I_CONST)
	return 1; //校验失败，返回0xff
	
	for(i=0;i<len;i++)
{
	Icm20602_WriteByte(initdata[i][0],initdata[i][1]);
	delay_ms(100);
}
	return 0;
}



static uint8_t Icm20602_GyroCalc(Icm20602Datadef *offset)
{
	uint16_t i=0,time=0;int32_t temp[3]={0};
	while(1)
	{
		if(Icm20602_GetIntStatus())
		{
			i++;	
			Icm20602_GetData(&cmd.Icm20602.Data.original);
			if(i%10==0)
			{
			time++;
			temp[0]-=cmd.Icm20602.Data.original.gx;
			temp[1]-=cmd.Icm20602.Data.original.gy;
			temp[2]-=cmd.Icm20602.Data.original.gz;
			}
		}
		if(time==calc_gyrotime)
			break;
	}
	temp[0]/=calc_gyrotime/2;
	temp[1]/=calc_gyrotime/2;
	temp[2]/=calc_gyrotime/2;
	//下面的50应该用方差来替代，找个合适的阈值
	if(temp[0]<100&&temp[1]<100&&temp[2]<100)
	{
		offset->gx=temp[0];
		offset->gy=temp[1];
		offset->gz=temp[2];
		Icm20602_WriteByte(ICM20602_XG_OFFS_USRH,(uint8_t)((int16_t)temp[0]>>8));
		Icm20602_WriteByte(ICM20602_XG_OFFS_USRL,(uint8_t)((int16_t)temp[0]&0xFF));
		Icm20602_WriteByte(ICM20602_YG_OFFS_USRH,(uint8_t)((int16_t)temp[1]>>8));
		Icm20602_WriteByte(ICM20602_YG_OFFS_USRL,(uint8_t)((int16_t)temp[1]&0xFF));
		Icm20602_WriteByte(ICM20602_ZG_OFFS_USRH,(uint8_t)((int16_t)temp[2]>>8));
		Icm20602_WriteByte(ICM20602_ZG_OFFS_USRL,(uint8_t)((int16_t)temp[2]&0xFF));
		return 0;//矫正成功
	}
	return 1;//矫正失败
}


static uint8_t Icm20602_AccelCalc(float ref[6][3])
{
	int i=0,time=0;
	uint8_t calc_axis=0,calc_lock[6]={0};
	float flashwrite[7][3];
	while(Icm20602_Get_Calibration_Status()==0)
	{
		
		int32_t temp[3]={0};
		
		if(Icm20602_GetIntStatus())
			{
				Icm20602_GetData(&cmd.Icm20602.Data.original);
			}
			
			while(Icm20602_Get_AccelCalibration_Status())
		{
			if(Icm20602_GetIntStatus())
			{
				i++;	
				Icm20602_GetData(&cmd.Icm20602.Data.original);
				if(i%10==0)
				{
				time++;
				temp[0]+=cmd.Icm20602.Data.original.ax;
				temp[1]+=cmd.Icm20602.Data.original.ay;
				temp[2]+=cmd.Icm20602.Data.original.az;
				}
			}
			if(time==calc_acctime)
			{
				temp[0]/=calc_acctime;
				temp[1]/=calc_acctime;
				temp[2]/=calc_acctime;
				if(temp[0]>ACCMAX)
				{
					ref[0][0]=temp[0];
					ref[0][1]=temp[1];
					ref[0][2]=temp[2];
					
					flashwrite[0][0]=temp[0];
					flashwrite[0][1]=temp[1];
					flashwrite[0][2]=temp[2];
					if(calc_lock[0]==0)
					{
						calc_axis++;
						calc_lock[0]=1;
					}
				}
				else if(temp[0]<-ACCMAX)
				{
					ref[1][0]=temp[0];
					ref[1][1]=temp[1];
					ref[1][2]=temp[2];
					
					flashwrite[1][0]=temp[0];
					flashwrite[1][1]=temp[1];
					flashwrite[1][2]=temp[2];
					if(calc_lock[1]==0)
					{
						calc_axis++;
						calc_lock[1]=1;
					}
				}
				if(temp[1]>ACCMAX)
				{
					ref[2][0]=temp[0];
					ref[2][1]=temp[1];
					ref[2][2]=temp[2];
					
					flashwrite[2][0]=temp[0];
					flashwrite[2][1]=temp[1];
					flashwrite[2][2]=temp[2];
					if(calc_lock[2]==0)
					{
						calc_axis++;
						calc_lock[2]=1;
					}
				}
				else if(temp[1]<-ACCMAX)
				{
					ref[3][0]=temp[0];
					ref[3][1]=temp[1];
					ref[3][2]=temp[2];
					
					flashwrite[3][0]=temp[0];
					flashwrite[3][1]=temp[1];
					flashwrite[3][2]=temp[2];
					if(calc_lock[3]==0)
					{
						calc_axis++;
						calc_lock[3]=1;
					}
				}
				if(temp[2]>ACCMAX)
				{
					ref[4][0]=temp[0];
					ref[4][1]=temp[1];
					ref[4][2]=temp[2];
					
					flashwrite[4][0]=temp[0];
					flashwrite[4][1]=temp[1];
					flashwrite[4][2]=temp[2];
					if(calc_lock[4]==0)
					{
						calc_axis++;
						calc_lock[4]=1;
					}
				}
				else if(temp[2]<-ACCMAX)
				{
					ref[5][0]=temp[0];
					ref[5][1]=temp[1];
					ref[5][2]=temp[2];
					
					flashwrite[5][0]=temp[0];
					flashwrite[5][1]=temp[1];
					flashwrite[5][2]=temp[2];
					if(calc_lock[5]==0)
					{
						calc_axis++;
						calc_lock[5]=1;
					}
				}
				i=0;
				time=0;
				Icm20602_Set_AccelCalibration_Status(0);
			}
							
		}
		if(calc_axis==6)
		{
			Icm20602_Set_Calibration_Status(1);
			flashwrite[6][0]=flashwrite[6][2]=2018;
			flashwrite[6][1]=Icm20602_Get_Calibration_Status();
			Bsp_Flash_Write(ADDR_FLASH_SECTOR_11,(uint8_t *)flashwrite,sizeof(flashwrite));
		}
		
	}
	float Aoffset[3]={0};
	Icm20602_calculate_calibration_values(cmd.Icm20602.calibrate.ref, cmd.Icm20602.calibrate.Crossaxis,Aoffset,8192.0f);
	cmd.Icm20602.calibrate.offset.ax=Aoffset[0];
	cmd.Icm20602.calibrate.offset.ay=Aoffset[1];
	cmd.Icm20602.calibrate.offset.az=Aoffset[2];
	
	return 0;
}

void Icm20602_OffsetInit(void)
{
	Icm20602_GyroCalc(&cmd.Icm20602.calibrate.offset);
	Icm20602_AccelCalc(cmd.Icm20602.calibrate.ref);
}
void Icm20602_CrossaxisTransformation(float crossaxis_inv[9],Icm20602Datadef *m,Icm20602Datadef *o)
{

	float outputtmp[3];

  outputtmp[0] = m->ax * crossaxis_inv[0] +
                 m->ay * crossaxis_inv[1] +
                 m->az * crossaxis_inv[2];

  outputtmp[1] = m->ax * crossaxis_inv[3] +
                 m->ay * crossaxis_inv[4] +
                 m->az * crossaxis_inv[5];
    
  outputtmp[2] = m->ax * crossaxis_inv[6] +
                 m->ay * crossaxis_inv[7] +
                 m->az * crossaxis_inv[8];
	o->ax= (short)(outputtmp[0])-cmd.Icm20602.calibrate.offset.ax;
  o->ay= (short)(outputtmp[1])-cmd.Icm20602.calibrate.offset.ay;
  o->az= (short)(outputtmp[2])-cmd.Icm20602.calibrate.offset.az;
//	o->ax= (short)(outputtmp[0]);
//  o->ay= (short)(outputtmp[1]);
//  o->az= (short)(outputtmp[2]);
}



uint8_t Icm20602_DataLimit(Icm20602Datadef *data)
{
	if(data->gx<ICM20602LIMIT_MIN&&data->gx>(-ICM20602LIMIT_MIN))
		data->gx=0;
	if(data->gy<ICM20602LIMIT_MIN&&data->gy>(-ICM20602LIMIT_MIN))
		data->gy=0;
	if(data->gz<ICM20602LIMIT_MIN&&data->gz>(-ICM20602LIMIT_MIN))
		data->gz=0;
	return 0;
}

void Icm20602_GetRegisterData(uint8_t *data)
{
	Icm20602_ReadBytes(ICM20602_ACCEL_XOUT_H,data,14);
}


void Icm20602_GetData(Icm20602Datadef *icmdata)
{
	uint8_t data[14];
	Icm20602_GetRegisterData(data);
	icmdata->ax=data[0]<<8|data[1];
	icmdata->ay=data[2]<<8|data[3];
	icmdata->az=data[4]<<8|data[5];
	icmdata->temp=data[6]<<8|data[7];
	
	icmdata->gx=(data[8]<<8|data[9]);
	icmdata->gy=(data[10]<<8|data[11]);
	icmdata->gz=(data[12]<<8|data[13]);
	if(icmdata->ax>8192)
		icmdata->ax=8192;
	if(icmdata->ax<-8192)
		icmdata->ax=-8192;
	
	if(icmdata->ay>8192)
		icmdata->ay=8192;
	if(icmdata->ay<-8192)
		icmdata->ay=-8192;
	
	if(icmdata->az>8192)
		icmdata->az=8192;
	if(icmdata->az<-8192)
		icmdata->az=-8192;

//	icmdata->gx=(data[8]<<8|data[9])+offset.gx;
//	icmdata->gy=(data[10]<<8|data[11])+offset.gy;
//	icmdata->gz=(data[12]<<8|data[13])+offset.gz;
	
}

#define Filter_time 100
#define filter_num 1
#define filter_rate 0.5
void Icm20602DataFilter(Icm20602Datadef *data,Icm20602Datadef *out)
{
//	out->ax=((Filter_time-filter_num)*out->ax+filter_num*data->ax)/Filter_time;
//	out->ay=((Filter_time-filter_num)*out->ay+filter_num*data->ay)/Filter_time;
//	out->az=((Filter_time-filter_num)*out->az+filter_num*data->az)/Filter_time;
	
	out->ax=data->ax;
	out->ay=data->ay;
	out->az=data->az;
	
	out->gx=(1-filter_rate)*out->gx+filter_rate*data->gx;
	out->gy=(1-filter_rate)*out->gy+filter_rate*data->gy;
	out->gz=(1-filter_rate)*out->gz+filter_rate*data->gz;
	
//	out->gx=data->gx;
//	out->gy=data->gy;
//	out->gz=data->gz;
	
	out->temp=((Filter_time-1)*out->temp+data->temp)/Filter_time;
}


void ICM20602_DataUpdate(void)
{
		Icm20602_GetData(&cmd.Icm20602.Data.original);
		Icm20602_CrossaxisTransformation(cmd.Icm20602.calibrate.Crossaxis,&cmd.Icm20602.Data.original,&cmd.Icm20602.Data.original);
		Icm20602DataFilter(&cmd.Icm20602.Data.original,&cmd.Icm20602.Data.calc);
		
		Icm20602_DataLimit(&cmd.Icm20602.Data.calc);
	
		Icm20602_SetDataStatus(1);
	
		Monitor_Set(&cmd.Icm20602.monitor);
}



void Icm20602_SetDataStatus(uint8_t x)
{
	if(x)
	cmd.Icm20602.Data.dataStatus=1;
	else
	cmd.Icm20602.Data.dataStatus=0;
}
uint8_t Icm20602_GetDataStatus(void)
{
	return cmd.Icm20602.Data.dataStatus;
}

uint8_t Icm20602_GetIntStatus(void)
{
	uint8_t a;
	Icm20602_ReadByte(ICM20602_INT_STATUS,&a);
	return a&0x01;
}

Icm20602Status Icm20602_GetStatus(void)
{
	return cmd.Icm20602.status;
}
void Icm20602_SetStatus(Icm20602Status  x)
{
	cmd.Icm20602.status=x;
}
void Icm20602_Set_Calibration_Status(uint8_t  x)
{
	if(x)
		cmd.Icm20602.calibrate.status=1;
	else
		cmd.Icm20602.calibrate.status=0;
}
uint8_t Icm20602_Get_Calibration_Status(void)
{
	return cmd.Icm20602.calibrate.status;
}
void Icm20602_Set_AccelCalibration_Status(uint8_t  x)
{
	if(x)
		cmd.Icm20602.calibrate.AccelOn=1;
	else
		cmd.Icm20602.calibrate.AccelOn=0;
}
uint8_t Icm20602_Get_AccelCalibration_Status(void)
{
	return cmd.Icm20602.calibrate.AccelOn;
}
