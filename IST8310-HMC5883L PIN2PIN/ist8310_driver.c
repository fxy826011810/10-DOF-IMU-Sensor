//IST8310 driver
#include "IST8310_driver.h"
#include <stdint.h>
#include <stdio.h>

extern void ist_i2c_read(uint8_t slaveAddr, uint8_t reg, uint8_t length, uint8_t* data);
extern void ist_i2c_write(uint8_t slaveAddr, uint8_t reg, uint8_t data);
extern void ist_delay(uint32_t milliseconds);

#ifdef IST_CROSS_AXIS_CALI
#ifdef IST_CROSS_AXIS_CALI_FLOAT
float crossaxis_inv[9];
#else
int64_t crossaxis_inv[9];
#endif
int32_t crossaxis_det[1];
//crossaxis cali_sensitivity and bit shift setting
#define OTPsensitivity (330)
#define crossaxisinv_bitshift (16)
#endif
#define SMA_WINDOW 2
int16_t pre_xyz[SMA_WINDOW-1][3] = {0};

int16_t SMA(int16_t raw, int position);
void ist8310_MergeHighLowData(uint8_t *raw, int16_t *xyz);
#ifdef IST_CROSS_AXIS_CALI
void ist8310_CrossaxisTransformation(int16_t *xyz);
void ist8310_Crossaxis_Matrix(int bitshift, int enable);
#endif

void ist8310_Init(){
    ist_delay(50);  //sensor initial time : 50 milliseconds
    ist_i2c_write(IST8310_SLA, IST8310_REG_AVGCNTL, 0x24); //average 16 times
    ist_i2c_write(IST8310_SLA, IST8310_REG_SELECTION_REG, 0xC0); //low-power mode
#ifdef IST_CROSS_AXIS_CALI
    int crossaxis_enable = 0;
    char cross_mask[1];
    uint8_t wbuffer[2];

    cross_mask[0]= 0xFF;
    ist_i2c_read(IST8310_SLA, IST8310_REG_XX_CROSS_L, 2, wbuffer);
	
    if((wbuffer[0] == cross_mask[0]) && (wbuffer[1] == cross_mask[0]))
        crossaxis_enable = 0;
    else
        crossaxis_enable = 1;

    ist8310_Crossaxis_Matrix(crossaxisinv_bitshift, crossaxis_enable);
#endif
}

void ist8310_GetXYZ(int16_t *xyz){
    uint8_t raw[IST8310_DATA_NUM];

    //Write ODR to force mode
    ist_i2c_write(IST8310_SLA, IST8310_REG_CNTRL1, IST8310_ODR_MODE);
    ist_delay(6);  //sensor get data : 6 milliseconds
    
    //Read 6 reg data from x high&low to z
    ist_i2c_read(IST8310_SLA, IST8310_REG_DATAX, IST8310_DATA_NUM, raw);
    //Combine High Low byte
    ist8310_MergeHighLowData(raw, xyz);
#ifdef IST_CROSS_AXIS_CALI
    ist8310_CrossaxisTransformation(xyz);
#endif
}

void ist8310_MergeHighLowData(uint8_t *raw, int16_t *xyz){
    xyz[0] = (((int16_t)raw[1]) << 8) | raw[0];
//    xyz[0] = SMA(xyz[0], 0);
    
    xyz[1] = (((int16_t)raw[3]) << 8) | raw[2];
//    xyz[1] = SMA(xyz[1], 1);

    xyz[2] = (((int16_t)raw[5]) << 8) | raw[4];
//    xyz[2] = SMA(xyz[2], 2);
}

int16_t SMA(int16_t raw, int position)
{
    int16_t t_raw = raw;
    int16_t value = 0;
    int i;
    for(i = 0; i < SMA_WINDOW - 1; i++)
    {
        value += pre_xyz[i][position];
    }
    value += t_raw;
    value /= SMA_WINDOW;
    
    //shift 1 position
    for(i = 0; i < SMA_WINDOW - 2; i++)
    {
        pre_xyz[i][position] = pre_xyz[i+1][position];
    }
    pre_xyz[SMA_WINDOW-2][position] = t_raw;
    
    return value;
}

#ifdef IST_CROSS_AXIS_CALI
void ist8310_CrossaxisTransformation(int16_t *xyz){
	int i = 0;
#ifdef IST_CROSS_AXIS_CALI_FLOAT
	float outputtmp[3];
#else
	int64_t outputtmp[3];
#endif
    for( i = 0; i < 9; i++){
        if(crossaxis_inv[i]!=0)
          break;
        if(i == 8)
          ist8310_Init();
    }

    outputtmp[0] = xyz[0] * crossaxis_inv[0] +
                   xyz[1] * crossaxis_inv[1] +
                   xyz[2] * crossaxis_inv[2];

    outputtmp[1] = xyz[0] * crossaxis_inv[3] +
                   xyz[1] * crossaxis_inv[4] +
                   xyz[2] * crossaxis_inv[5];
    
    outputtmp[2] = xyz[0] * crossaxis_inv[6] +
                   xyz[1] * crossaxis_inv[7] +
                   xyz[2] * crossaxis_inv[8];
#ifdef IST_CROSS_AXIS_CALI_FLOAT
    xyz[0]= (short)(outputtmp[0]);
    xyz[1]= (short)(outputtmp[1]);
    xyz[2]= (short)(outputtmp[2]);
#else
    for (i=0; i<IST8310_AXES_NUM; i++)
    {
        outputtmp[i] = outputtmp[i] / (*crossaxis_det);
    }
    xyz[0]= (short)(outputtmp[0] >> crossaxisinv_bitshift);
    xyz[1]= (short)(outputtmp[1] >> crossaxisinv_bitshift);
    xyz[2]= (short)(outputtmp[2] >> crossaxisinv_bitshift);
#endif
}
#endif

#ifdef IST_CROSS_AXIS_CALI
void ist8310_Crossaxis_Matrix(int bitshift, int enable)
{
//    int ret;
    int i = 0;
    uint8_t crossxbuf[6];
    uint8_t crossybuf[6];
    uint8_t crosszbuf[6];
    short OTPcrossaxis[9] = {0};
#ifdef IST_CROSS_AXIS_CALI_FLOAT
    float inv[9] = {0};
#else 
    int64_t inv[9] = {0};
#endif
    
    if (enable == 0)
    {
DET_eql_0:
#ifdef IST_CROSS_AXIS_CALI_FLOAT
        *crossaxis_inv = 1;
#else
        *crossaxis_inv = (1<<bitshift);
#endif
        *(crossaxis_inv+1) = 0;
        *(crossaxis_inv+2) = 0;
        *(crossaxis_inv+3) = 0;
#ifdef IST_CROSS_AXIS_CALI_FLOAT
        *(crossaxis_inv+4) = 1;
#else
        *(crossaxis_inv+4) = (1<<bitshift);
#endif
        *(crossaxis_inv+5) = 0;
        *(crossaxis_inv+6) = 0;
        *(crossaxis_inv+7) = 0;
#ifdef IST_CROSS_AXIS_CALI_FLOAT
        *(crossaxis_inv+8) = 1;
#else
        *(crossaxis_inv+8) = (1<<bitshift); 
#endif
        *crossaxis_det = 1;

//        for (i=0; i<9; i++)
//        {
//            printf("*(crossaxis_inv + %d) = %lld\n", i, *(crossaxis_inv+i));
//        }
//        printf("det = %d\n",*crossaxis_det);
        return;
    }    
    else
    {
        ist_i2c_read(IST8310_SLA, IST8310_REG_XX_CROSS_L, 6, crossxbuf);

        ist_i2c_read(IST8310_SLA, IST8310_REG_YX_CROSS_L, 6, crossybuf);

        ist_i2c_read(IST8310_SLA, IST8310_REG_ZX_CROSS_L, 6, crosszbuf);

        OTPcrossaxis[0] = ((int16_t) crossxbuf[1]) << 8 | crossxbuf[0];
        OTPcrossaxis[3] = ((int16_t) crossxbuf[3]) << 8 | crossxbuf[2];
        OTPcrossaxis[6] = ((int16_t) crossxbuf[5]) << 8 | crossxbuf[4];
        OTPcrossaxis[1] = ((int16_t) crossybuf[1]) << 8 | crossybuf[0];
        OTPcrossaxis[4] = ((int16_t) crossybuf[3]) << 8 | crossybuf[2];
        OTPcrossaxis[7] = ((int16_t) crossybuf[5]) << 8 | crossybuf[4];
        OTPcrossaxis[2] = ((int16_t) crosszbuf[1]) << 8 | crosszbuf[0];
        OTPcrossaxis[5] = ((int16_t) crosszbuf[3]) << 8 | crosszbuf[2];
        OTPcrossaxis[8] = ((int16_t) crosszbuf[5]) << 8 | crosszbuf[4];
        *crossaxis_det = ((int32_t)OTPcrossaxis[0])*OTPcrossaxis[4]*OTPcrossaxis[8] +
               ((int32_t)OTPcrossaxis[1])*OTPcrossaxis[5]*OTPcrossaxis[6] +
               ((int32_t)OTPcrossaxis[2])*OTPcrossaxis[3]*OTPcrossaxis[7] -
               ((int32_t)OTPcrossaxis[0])*OTPcrossaxis[5]*OTPcrossaxis[7] -
               ((int32_t)OTPcrossaxis[2])*OTPcrossaxis[4]*OTPcrossaxis[6] -
               ((int32_t)OTPcrossaxis[1])*OTPcrossaxis[3]*OTPcrossaxis[8];
        
        if (*crossaxis_det == 0) {
            goto DET_eql_0;
        }
#ifdef IST_CROSS_AXIS_CALI_FLOAT
        inv[0] = (float)OTPcrossaxis[4] * OTPcrossaxis[8] - (float)OTPcrossaxis[5] * OTPcrossaxis[7];
        inv[1] = (float)OTPcrossaxis[2] * OTPcrossaxis[7] - (float)OTPcrossaxis[1] * OTPcrossaxis[8];
        inv[2] = (float)OTPcrossaxis[1] * OTPcrossaxis[5] - (float)OTPcrossaxis[2] * OTPcrossaxis[4];
        inv[3] = (float)OTPcrossaxis[5] * OTPcrossaxis[6] - (float)OTPcrossaxis[3] * OTPcrossaxis[8];
        inv[4] = (float)OTPcrossaxis[0] * OTPcrossaxis[8] - (float)OTPcrossaxis[2] * OTPcrossaxis[6];
        inv[5] = (float)OTPcrossaxis[2] * OTPcrossaxis[3] - (float)OTPcrossaxis[0] * OTPcrossaxis[5];
        inv[6] = (float)OTPcrossaxis[3] * OTPcrossaxis[7] - (float)OTPcrossaxis[4] * OTPcrossaxis[6];
        inv[7] = (float)OTPcrossaxis[1] * OTPcrossaxis[6] - (float)OTPcrossaxis[0] * OTPcrossaxis[7];
        inv[8] = (float)OTPcrossaxis[0] * OTPcrossaxis[4] - (float)OTPcrossaxis[1] * OTPcrossaxis[3];
        
        for (i=0; i<9; i++) {
            crossaxis_inv[i] = inv[i] * OTPsensitivity/(*crossaxis_det);
        }
#else        
        inv[0] = (int64_t)OTPcrossaxis[4] * OTPcrossaxis[8] - (int64_t)OTPcrossaxis[5] * OTPcrossaxis[7];
        inv[1] = (int64_t)OTPcrossaxis[2] * OTPcrossaxis[7] - (int64_t)OTPcrossaxis[1] * OTPcrossaxis[8];
        inv[2] = (int64_t)OTPcrossaxis[1] * OTPcrossaxis[5] - (int64_t)OTPcrossaxis[2] * OTPcrossaxis[4];
        inv[3] = (int64_t)OTPcrossaxis[5] * OTPcrossaxis[6] - (int64_t)OTPcrossaxis[3] * OTPcrossaxis[8];
        inv[4] = (int64_t)OTPcrossaxis[0] * OTPcrossaxis[8] - (int64_t)OTPcrossaxis[2] * OTPcrossaxis[6];
        inv[5] = (int64_t)OTPcrossaxis[2] * OTPcrossaxis[3] - (int64_t)OTPcrossaxis[0] * OTPcrossaxis[5];
        inv[6] = (int64_t)OTPcrossaxis[3] * OTPcrossaxis[7] - (int64_t)OTPcrossaxis[4] * OTPcrossaxis[6];
        inv[7] = (int64_t)OTPcrossaxis[1] * OTPcrossaxis[6] - (int64_t)OTPcrossaxis[0] * OTPcrossaxis[7];
        inv[8] = (int64_t)OTPcrossaxis[0] * OTPcrossaxis[4] - (int64_t)OTPcrossaxis[1] * OTPcrossaxis[3];
        
        for (i=0; i<9; i++) {
            crossaxis_inv[i] = (inv[i] << bitshift) * OTPsensitivity;
        }
#endif
    }
    
    return;
}
#endif