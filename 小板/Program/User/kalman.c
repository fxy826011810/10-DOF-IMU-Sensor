/** *****************************************************************************************
  * @file    kalman.c                                                                                                                                      *
  * @author  Liu heng                                                                                                                                    *
  * @version V1.0.0                                                                                                                                      *
  * @date    27-August-2013                                                                                                                         *
  * @brief   Ò»Î¬¿¨¶ûÂüÂË²¨Æ÷µÄ¾ßÌåÊµÏÖ¡£ÊµÏÖ¹ý³ÌÍêÈ«ÓëÓ²¼þÎÞ¹Ø£¬   *
  *   ¿ÉÖ±½Óµ÷ÓÃ£¬ÈÎÒâÒÆÖ²¡£                                                                                               *
  *   Ê¹ÓÃÊ±ÏÈ¶¨ÒåÒ»¸ökalmanÖ¸Õë£¬È»ºóµ÷ÓÃkalmanCreate()´´½¨Ò»¸öÂË²¨Æ÷£**
  *   Ã¿´Î¶ÁÈ¡µ½´«¸ÐÆ÷Êý¾Ýºó¼´¿Éµ÷ÓÃKalmanFilter()À´¶ÔÊý¾Ý½øÐÐÂË²¨¡£               *
  *****************************************************************************************
  *                          Ê¹ÓÃÊ¾Àý                                                     *
  *          kalman p;                                                                   *
  *          float SersorData;                                                            *
  *          kalmanCreate(&p,20,200);                                                  *
  *          while(1)                                                                     *
  *          {                                                                            *
  *             SersorData = sersor();                                                    *
  *             SersorData = KalmanFilter(&p,SersorData);                                  *
  *             printf("%2.2f",SersorData);                                               *
  *          }                                                                            *
  *****************************************************************************************
  *          MPU6050µÄ¿¨¶ûÂüÂË²¨Æ÷²Î¿¼²ÎÊý Q£º10 R£º400                                   *
  *****************************************************************************************/

#include "kalman.h"
#include "main.h"
/**
  * @name   kalmanCreate
  * @brief  ´´½¨Ò»¸ö¿¨¶ûÂüÂË²¨Æ÷
  * @param  p:  ÂË²¨Æ÷
  *         T_Q:ÏµÍ³ÔëÉùÐ­·½²î
  *         T_R:²âÁ¿ÔëÉùÐ­·½²î
  *         
  * @retval none
  */
	kalman Kgx,Kgy,Kgz,Kax,Kay,Kaz;
	
void kalmanInit(void)	
{
	kalmanCreate(&Kgx,10,400);
	kalmanCreate(&Kgy,10,400);
	kalmanCreate(&Kgz,10,400);
	kalmanCreate(&Kax,10,400);
	kalmanCreate(&Kay,10,400);
	kalmanCreate(&Kaz,10,400);
}	

void kalmanUpdate(void)	
{
	cmd.Icm20602->Data.calc.gx=(int16_t)KalmanFilter(&Kgx,cmd.Icm20602->Data.calc.gx);
	cmd.Icm20602->Data.calc.gy=(int16_t)KalmanFilter(&Kgy,cmd.Icm20602->Data.calc.gy);
	cmd.Icm20602->Data.calc.gz=(int16_t)KalmanFilter(&Kgz,cmd.Icm20602->Data.calc.gz);
	cmd.Icm20602->Data.calc.ax=(int16_t)KalmanFilter(&Kax,cmd.Icm20602->Data.calc.ax);
	cmd.Icm20602->Data.calc.ay=(int16_t)KalmanFilter(&Kay,cmd.Icm20602->Data.calc.ay);
	cmd.Icm20602->Data.calc.az=(int16_t)KalmanFilter(&Kaz,cmd.Icm20602->Data.calc.az);
}
	
	
void kalmanCreate(kalman *p,float T_Q,float T_R)
{
    //kalman* p = ( kalman*)malloc(sizeof( kalman));
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->H = 1;
    p->X_mid = p->X_last;
    //return p;
}

/**
  * @name   KalmanFilter
  * @brief  ¿¨¶ûÂüÂË²¨Æ÷
  * @param  p:  ÂË²¨Æ÷
  *         dat:´ýÂË²¨Êý¾Ý
  * @retval ÂË²¨ºóµÄÊý¾Ý
  */

float KalmanFilter(kalman* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //×´Ì¬¸üÐÂ
    p->X_last = p->X_now;
    return p->X_now;
}
