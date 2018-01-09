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
uint8_t DataStatus;
}ahrs_t;
void AHRS_Init(ahrs_t *ahrs);
void _6AxisAHRSupdate(Icm20602Datadef *imu,ahrs_t *ahrs);
void _9AxisAHRSupdate(Icm20602Datadef *imu,magDatadef *m,ahrs_t *ahrs);
#endif