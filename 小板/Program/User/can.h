#ifndef __CAN_H
#define __CAN_H
#include "stm32f4xx.h"
#define __CAN_EXT extern


void Bsp_Can_Init(void);

void Can_AngleSend(CAN_TypeDef* CANx, float angle[3]);
void gm_senddata(CAN_TypeDef* CANx, int num1, int num2);
void gyro_senddata(void);


#endif

