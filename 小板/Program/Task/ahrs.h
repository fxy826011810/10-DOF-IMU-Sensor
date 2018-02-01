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
void AHRS_Init(ahrs_t *ahrs);
void AHRS_Update(void);

uint8_t AHRS_GetDataStatus(void);
void AHRS_SetDataStatus(uint8_t x);

#endif
