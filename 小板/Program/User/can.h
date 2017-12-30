#ifndef __CAN_H
#define __CAN_H
#include "stm32f4xx.h"
#define __CAN_EXT extern


void Bsp_Can_Init(void);

void cm_senddata(CAN_TypeDef* CANx, int num1, int num2, int num3, int num4);
void gm_senddata(CAN_TypeDef* CANx, int num1, int num2);
void gyro_senddata(void);


#endif

