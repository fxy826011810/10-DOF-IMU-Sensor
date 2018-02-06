#include "ahrs.h"
#include "icm20602.h"
#include "ist8310.h"
#include "math.h"
#include "tim.h"
#include "main.h"
#include "kalman.h"
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void AHRS_Init(ahrs_t *ahrs)
{
	ahrs->dataStatus=0;
	ahrs->q[0]=1.0f;
	ahrs->q[1]=0.0f;
	ahrs->q[2]=0.0f;
	ahrs->q[3]=0.0f;
	
	ahrs->angle[0]=0.0f;
	ahrs->angle[1]=0.0f;
	ahrs->angle[2]=0.0f;
	
	ahrs->kp[0]=2.0f;
	ahrs->ki[0]=0.01f;
	
	ahrs->kp[1]=4.0f;
	ahrs->ki[1]=0.01f;
}

float g;
#define M_PI 3.1415926535f
uint32_t lastUpdate, now; // �������ڼ��� ��λ us 
float exInt, eyInt, ezInt;  // ������
static void _6AxisAHRSupdate(Icm20602Datadef *imu,ahrs_t *ahrs) 
{
		float norm;
    float vx, vy, vz;//vxyzΪ��������ϵת����������ϵ�ķ���
    float ex, ey, ez;//����ת�������ͼ��ٶȼƸ���ķ����������Ϊ���
    float tempq0,tempq1,tempq2,tempq3;
		float gx, gy, gz, ax, ay, az,halfT; 
    float q0q0 = ahrs->q[0]*ahrs->q[0];
    float q0q1 = ahrs->q[0]*ahrs->q[1];
    float q0q2 = ahrs->q[0]*ahrs->q[2];
    float q0q3 = ahrs->q[0]*ahrs->q[3];
    float q1q1 = ahrs->q[1]*ahrs->q[1];
    float q1q2 = ahrs->q[1]*ahrs->q[2];
    float q1q3 = ahrs->q[1]*ahrs->q[3];
    float q2q2 = ahrs->q[2]*ahrs->q[2];   
    float q2q3 = ahrs->q[2]*ahrs->q[3];
    float q3q3 = ahrs->q[3]*ahrs->q[3];   
		
		gx = (float)imu->gx/16.4f/57.3f;
    gy = (float)imu->gy/16.4f/57.3f;
    gz = (float)imu->gz/16.4f/57.3f;
		ax = (float)imu->ax/8192.0f*9.8f;
    ay = (float)imu->ay/8192.0f*9.8f;
    az = (float)imu->az/8192.0f*9.8f;

    now = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
		if(now>=lastUpdate)	
    {
        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//����ʱ��

    norm = invSqrt(ax*ax + ay*ay + az*az);    
		g=norm;		
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm; //�ѼӼƵ���ά����ת�ɵ�λ������

		vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
		
		ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)//�����PI����
    {
        exInt = exInt + ex * ahrs->ki[0] * halfT;
        eyInt = eyInt + ey * ahrs->ki[0] * halfT;	
        ezInt = ezInt + ez * ahrs->ki[0] * halfT;
        // �ò���������PI����������ƫ
        gx = gx + ahrs->kp[0]*ex + exInt;
        gy = gy + ahrs->kp[0]*ey + eyInt;
        gz = gz + ahrs->kp[0]*ez + ezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = ahrs->q[0] + (-ahrs->q[1]*gx - ahrs->q[2]*gy - ahrs->q[3]*gz)*halfT;
    tempq1 = ahrs->q[1] + (ahrs->q[0]*gx + ahrs->q[2]*gz - ahrs->q[3]*gy)*halfT;
    tempq2 = ahrs->q[2] + (ahrs->q[0]*gy - ahrs->q[1]*gz + ahrs->q[3]*gx)*halfT;
    tempq3 = ahrs->q[3] + (ahrs->q[0]*gz + ahrs->q[1]*gy - ahrs->q[2]*gx)*halfT;  

    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    ahrs->q[0] = tempq0 * norm;
    ahrs->q[1] = tempq1 * norm;
    ahrs->q[2] = tempq2 * norm;
    ahrs->q[3] = tempq3 * norm;
}
int8_t a=1,b=1,c=1;
static void _9AxisAHRSupdate(Icm20602Datadef *imu,magDatadef *m,ahrs_t *ahrs) 
{
		float norm;
    float hx, hy, hz, bx, bz;//hxyzΪ������ϵת����������ϵ�ĸ�������
    float vx, vy, vz, wx, wy, wz;//vxyzΪ��������ϵת����������ϵ�ķ���
    float ex, ey, ez;//����ת�������ͼ��ٶȼƸ���ķ����������Ϊ���
    float tempq0,tempq1,tempq2,tempq3;
		float gx, gy, gz, ax, ay, az, mx, my, mz,halfT; 
    float q0q0 = ahrs->q[0]*ahrs->q[0];
    float q0q1 = ahrs->q[0]*ahrs->q[1];
    float q0q2 = ahrs->q[0]*ahrs->q[2];
    float q0q3 = ahrs->q[0]*ahrs->q[3];
    float q1q1 = ahrs->q[1]*ahrs->q[1];
    float q1q2 = ahrs->q[1]*ahrs->q[2];
    float q1q3 = ahrs->q[1]*ahrs->q[3];
    float q2q2 = ahrs->q[2]*ahrs->q[2];   
    float q2q3 = ahrs->q[2]*ahrs->q[3];
    float q3q3 = ahrs->q[3]*ahrs->q[3];    
		
		gx = (float)imu->gx/16.4f/57.3f;
    gy = (float)imu->gy/16.4f/57.3f;
    gz = (float)imu->gz/16.4f/57.3f;
    
    mx = (float)(m->my)*a;
    my = (float)(m->mx)*b;
    mz = (float)(m->mz)*c;

		ax = (float)imu->ax/8192.0f*9.8f;
    ay = (float)imu->ay/8192.0f*9.8f;
    az = (float)imu->az/8192.0f*9.8f;

    now = Get_Time_Micros();  //��ȡʱ�� ��λ��us   
		if(now>=lastUpdate)	
    {
        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//����ʱ��
    //������ƽ�����㷨
    norm = invSqrt(ax*ax + ay*ay + az*az);
		g=norm;		
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm; //�ѼӼƵ���ά����ת�ɵ�λ������

    norm = invSqrt(mx*mx + my*my + mz*mz);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm; 
    // compute reference direction of flux
    hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
    hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
    hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz; 
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
    wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
    wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)//�����PI����
    {
        exInt = exInt + ex * ahrs->ki[1] * halfT;
        eyInt = eyInt + ey * ahrs->ki[1] * halfT;	
        ezInt = ezInt + ez * ahrs->ki[1] * halfT;
        // �ò���������PI����������ƫ
        gx = gx + ahrs->kp[1]*ex + exInt;
        gy = gy + ahrs->kp[1]*ey + eyInt;
        gz = gz + ahrs->kp[1]*ez + ezInt;
    }
    // ��Ԫ��΢�ַ���
    tempq0 = ahrs->q[0] + (-ahrs->q[1]*gx - ahrs->q[2]*gy - ahrs->q[3]*gz)*halfT;
    tempq1 = ahrs->q[1] + (ahrs->q[0]*gx + ahrs->q[2]*gz - ahrs->q[3]*gy)*halfT;
    tempq2 = ahrs->q[2] + (ahrs->q[0]*gy - ahrs->q[1]*gz + ahrs->q[3]*gx)*halfT;
    tempq3 = ahrs->q[3] + (ahrs->q[0]*gz + ahrs->q[1]*gy - ahrs->q[2]*gx)*halfT;  

    // ��Ԫ���淶��
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    ahrs->q[0] = tempq0 * norm;
    ahrs->q[1] = tempq1 * norm;
    ahrs->q[2] = tempq2 * norm;
    ahrs->q[3] = tempq3 * norm;

}
static void AHRS_Q_To_Angle(ahrs_t *ahrs)
{
		ahrs->angle[0] = -atan2(2 * ahrs->q[1] * ahrs->q[2] + 2 * ahrs->q[0] * ahrs->q[3], -2 * ahrs->q[2] * ahrs->q[2] - 2 * ahrs->q[3] * ahrs->q[3] + 1) * 180 / M_PI;
		ahrs->angle[1] = -asin(-2 * ahrs->q[1] * ahrs->q[3] + 2 * ahrs->q[0] * ahrs->q[2]) * 180 / M_PI; // pitch    -pi/2    --- pi/2 
		ahrs->angle[2] = atan2(2 * ahrs->q[2] * ahrs->q[3] + 2 * ahrs->q[0] * ahrs->q[1], -2 * ahrs->q[1] * ahrs->q[1] - 2 * ahrs->q[2] * ahrs->q[2] + 1) * 180 / M_PI;
}
void AHRS_Update(void)
{
	kalmanUpdate();
	if(IST8310_GetStatus()==1&&IST8310_GetDataStatus()==1&&Icm20602_GetDataStatus()==1&&Icm20602_GetStatus()!=Lost)
	{
		_9AxisAHRSupdate(&cmd.Icm20602.Data.calc,&cmd.Ist8310.Data.calc,&cmd.ahrs);
		Icm20602_SetDataStatus(0);
		IST8310_SetDataStatus(0);
		AHRS_SetDataStatus(1);
	}
	else if(Icm20602_GetDataStatus()==1&&IST8310_GetDataStatus()==0&&Icm20602_GetStatus()!=Lost)
	{
		_6AxisAHRSupdate(&cmd.Icm20602.Data.calc,&cmd.ahrs);
		Icm20602_SetDataStatus(0);
		AHRS_SetDataStatus(1);
	}
	if(AHRS_GetDataStatus())
	{
		AHRS_Q_To_Angle(&cmd.ahrs);
	}
}	
	
uint8_t AHRS_GetDataStatus(void)
{
	return cmd.ahrs.dataStatus;
}
void AHRS_SetDataStatus(uint8_t x)
{
	cmd.ahrs.dataStatus=x;
}
