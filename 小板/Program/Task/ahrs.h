#ifndef __AHRS_H__
#define __AHRS_H__
#include "icm20602.h"
#include "ist8310.h"
typedef struct 
{
float q[4];
float angle[3];
float kp[2];
float ki[2];
uint8_t dataStatus;
}ahrs_t;
void AHRS_Init(void);
void _6AxisAHRSupdate(Icm20602Datadef *imu,ahrs_t *ahrs);
void _9AxisAHRSupdate(Icm20602Datadef *imu,magDatadef *m,ahrs_t *ahrs);
void AHRS_Q_To_Angle(ahrs_t *ahrs);

uint8_t AHRS_GetDataStatus(void);
void AHRS_SetDataStatus(uint8_t x);

#endif
