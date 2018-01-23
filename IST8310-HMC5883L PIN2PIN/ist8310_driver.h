#ifndef IST8310_DRIVER_H
#define IST8310_DRIVER_H

#include "stdint.h"

//#define IST_CROSS_AXIS_CALI_FLOAT
#define IST_CROSS_AXIS_CALI
//Defines a register address of the IST8310
#define IST8310_REG_WIA				0x00	//Who I am
#define IST8310_REG_INFO			0x01	//More Info
#define IST8310_REG_DATAX			0x03	//Output Value x
#define IST8310_REG_DATAY			0x05	//Output Value y
#define IST8310_REG_DATAZ			0x07	//Output Value z
#define IST8310_REG_STAT1			0x02	//Status register
#define IST8310_REG_STAT2			0x09	//Status register
#define IST8310_REG_CNTRL1			0x0A	//Control setting register 1
#define IST8310_REG_CNTRL2			0x0B	//Control setting register 2
#define IST8310_REG_CNTRL3			0x0D	//Control setting register 3
#define IST8310_REG_OFFSET_START	0xDC	//Offset
#define IST8310_REG_SELECTION_REG   0x42    //Sensor Selection register
#define IST8310_REG_TEST_REG        0x40    //Chip Test register
#define IST8310_REG_AVGCNTL         0x41    //Average Control register
#define IST8310_REG_TUNING_REG      0x47    //Bandgap Tuning register
#define IST8310_SLA                 0x0C    //IST8310 Slave Address

#define IST8310_ODR_MODE            0x01    //Force mode
#define IST8310_DATA_NUM            6       //XYZ High&Low

#ifdef IST_CROSS_AXIS_CALI
/*---IST8310 cross-axis matrix Address-----------------danny-----*/
#define IST8310_REG_XX_CROSS_L       0x9C   //cross axis xx low byte
#define IST8310_REG_XX_CROSS_H       0x9D   //cross axis xx high byte
#define IST8310_REG_XY_CROSS_L       0x9E   //cross axis xy low byte
#define IST8310_REG_XY_CROSS_H       0x9F   //cross axis xy high byte
#define IST8310_REG_XZ_CROSS_L       0xA0   //cross axis xz low byte
#define IST8310_REG_XZ_CROSS_H       0xA1   //cross axis xz high byte

#define IST8310_REG_YX_CROSS_L       0xA2   //cross axis yx low byte
#define IST8310_REG_YX_CROSS_H       0xA3   //cross axis yx high byte
#define IST8310_REG_YY_CROSS_L       0xA4   //cross axis yy low byte
#define IST8310_REG_YY_CROSS_H       0xA5   //cross axis yy high byte
#define IST8310_REG_YZ_CROSS_L       0xA6   //cross axis yz low byte
#define IST8310_REG_YZ_CROSS_H       0xA7   //cross axis yz high byte

#define IST8310_REG_ZX_CROSS_L       0xA8   //cross axis zx low byte
#define IST8310_REG_ZX_CROSS_H       0xA9   //cross axis zx high byte
#define IST8310_REG_ZY_CROSS_L       0xAA   //cross axis zy low byte
#define IST8310_REG_ZY_CROSS_H       0xAB   //cross axis zy high byte
#define IST8310_REG_ZZ_CROSS_L       0xAC   //cross axis zz low byte
#define IST8310_REG_ZZ_CROSS_H       0xAD   //cross axis zz high byte

#define IST8310_AXIS_X            0
#define IST8310_AXIS_Y            1
#define IST8310_AXIS_Z            2
#define IST8310_AXES_NUM          3
#endif

void ist8310_GetXYZ(int16_t *xyz);
void ist8310_Init(void);
#endif
