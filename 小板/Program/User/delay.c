#include "main.h"
//void delay_ms(uint16_t t)
//{
//  u32 i;
//	SysTick->LOAD=21000*t;
//        SysTick->VAL=0;
//	SysTick->CTRL=0x01;
//        do
//        {
//          i=SysTick->CTRL;
//        }
//	while((i&0x01)&&!(i&0x10000));
//	SysTick->CTRL=0;
//        SysTick->VAL=0;
//	
//}

//void delay_us(uint16_t t)
//{
//  u32 i;
//	SysTick->LOAD=21*t;
//        SysTick->VAL=0;
//	SysTick->CTRL=0x01;
//        do
//        {  
//					i=SysTick->CTRL;
//        }
//	while((i&0x01)&&!(i&0x10000));
//	SysTick->CTRL=0;
//        SysTick->VAL=0;
//	
//}

 void delay_us(uint16_t t)
{
	uint32_t i=0,j=0;
	for(i=0;i<t;i++)
	{
		for(j=0;j<40;j++)
		{
			
		}
	}
}
 void delay_ms(uint16_t t)
{
		uint16_t i=0;
	for(i=0;i<t;i++)
	{
		delay_us(1000);
	}
}

