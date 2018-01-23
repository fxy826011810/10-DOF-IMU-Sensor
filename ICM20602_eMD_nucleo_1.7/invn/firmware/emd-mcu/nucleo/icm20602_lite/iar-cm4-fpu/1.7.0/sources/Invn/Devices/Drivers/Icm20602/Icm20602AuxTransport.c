/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "Icm20602Transport.h"
#include "Icm20602Serif.h"
#include "Icm20602Defs.h"
#include "Icm20602.h"

#if (MEMS_CHIP == HW_ICM20690)
void inv_icm20602_init_secondary(struct inv_icm20602 * s)
{
	s->secondary_state.slv_reg[0].addr = MPUREG_I2C_SLV0_ADDR;
	s->secondary_state.slv_reg[0].reg  = MPUREG_I2C_SLV0_REG;
	s->secondary_state.slv_reg[0].ctrl = MPUREG_I2C_SLV0_CTRL;
	s->secondary_state.slv_reg[0].d0   = MPUREG_I2C_SLV0_DO;

	s->secondary_state.slv_reg[1].addr = MPUREG_I2C_SLV1_ADDR;
	s->secondary_state.slv_reg[1].reg  = MPUREG_I2C_SLV1_REG;
	s->secondary_state.slv_reg[1].ctrl = MPUREG_I2C_SLV1_CTRL;
	s->secondary_state.slv_reg[1].d0   = MPUREG_I2C_SLV1_DO;

	s->secondary_state.slv_reg[2].addr = MPUREG_I2C_SLV2_ADDR;
	s->secondary_state.slv_reg[2].reg  = MPUREG_I2C_SLV2_REG;
	s->secondary_state.slv_reg[2].ctrl = MPUREG_I2C_SLV2_CTRL;
	s->secondary_state.slv_reg[2].d0   = MPUREG_I2C_SLV2_DO;

	/* Make sure that by default all channels are disabled 
	To not inherit from a previous configuration from a previous run*/
	inv_icm20602_secondary_stop_channel(s, 0);
	inv_icm20602_secondary_stop_channel(s, 1);
	inv_icm20602_secondary_stop_channel(s, 2);
}

/* the following functions are used for configuring the secondary devices */

/*
 * inv_configure_secondary_read(): set secondary registers for reading.
 * The chip must be set as bank 3 before calling.
 * This is derived from inv_read_secondary in linux...
 * for now, uses a very simple data struct for the registers
 *
 * index gives the mapping to the particular SLVx registers
 * addr is the physical address of the device to be accessed
 * reg is the device register we wish to access
 * len is the number of bytes to be read
 *
 */
int inv_icm20602_read_secondary(struct inv_icm20602 * s, int index, uint8_t addr, unsigned char reg, char len)
{
	int result = 0;
	unsigned char data;

	data = INV_MPU_BIT_I2C_READ | addr;
	result |= inv_icm20602_mems_write_reg(s, s->secondary_state.slv_reg[index].addr, 1, &data);

	data = reg;
	result |= inv_icm20602_mems_write_reg(s, s->secondary_state.slv_reg[index].reg, 1, &data);

	data = INV_MPU_BIT_SLV_EN | len;
	result |= inv_icm20602_mems_write_reg(s, s->secondary_state.slv_reg[index].ctrl, 1, &data);

	return result;
}

int inv_icm20602_execute_read_secondary(struct inv_icm20602 * s, int index, uint8_t addr, int reg, int len, uint8_t *d)
{
	int result = 0;
	unsigned char orig_power_state = inv_icm20602_get_chip_power_state(s);

	result |= inv_icm20602_set_chip_power_state(s, CHIP_AWAKE, 1);
	result |= inv_icm20602_set_chip_power_state(s, CHIP_LP_ENABLE, 0);
	result |= inv_icm20602_read_secondary(s, index, addr, reg, len);
	result |= inv_icm20602_secondary_enable_i2c(s);

	inv_icm20602_sleep(SECONDARY_INIT_WAIT);

	result |= inv_icm20602_secondary_disable_i2c(s);
	result |= inv_icm20602_mems_read_reg(s, MPUREG_EXT_SLV_SENS_DATA_00, len, d);
	result |= inv_icm20602_secondary_stop_channel(s, index);
	result |= inv_icm20602_set_chip_power_state(s, CHIP_LP_ENABLE, !!(orig_power_state & CHIP_LP_ENABLE));
	result |= inv_icm20602_set_chip_power_state(s, CHIP_AWAKE, !!(orig_power_state & CHIP_AWAKE));

	return result;
}

/*
 * inv_write_secondary(): set secondary registers for writing?.
 * The chip must be set as bank 3 before calling.
 * This is derived from inv_write_secondary in linux...
 * for now, uses a very simple data struct for the registers
 *
 * index gives the mapping to the particular SLVx registers
 * addr is the physical address of the device to be accessed
 * reg is the device register we wish to access
 * len is the number of bytes to be read
 *
 */
int inv_icm20602_write_secondary(struct inv_icm20602 * s, int index, uint8_t addr, unsigned char reg, char v)
{
	int result = 0;
	unsigned char data;

	data = (unsigned char)addr;
	result |= inv_icm20602_mems_write_reg(s, s->secondary_state.slv_reg[index].addr, 1, &data);

	data = reg;
	result |= inv_icm20602_mems_write_reg(s, s->secondary_state.slv_reg[index].reg, 1, &data);

	data = v;
	result |= inv_icm20602_mems_write_reg(s, s->secondary_state.slv_reg[index].d0, 1, &data);

	data = INV_MPU_BIT_SLV_EN | 1;
	result |= inv_icm20602_mems_write_reg(s, s->secondary_state.slv_reg[index].ctrl, 1, &data);

	return result;
}

int inv_icm20602_execute_write_secondary(struct inv_icm20602 * s, int index, uint8_t addr, int reg, uint8_t v)
{
	int result = 0;
	unsigned char orig_power_state = inv_icm20602_get_chip_power_state(s);

	result |= inv_icm20602_set_chip_power_state(s, CHIP_AWAKE, 1);
	result |= inv_icm20602_set_chip_power_state(s, CHIP_LP_ENABLE, 0);
	result |= inv_icm20602_write_secondary(s, index, addr, reg, v);
	result |= inv_icm20602_secondary_enable_i2c(s);

	inv_icm20602_sleep(SECONDARY_INIT_WAIT);

	result |= inv_icm20602_secondary_disable_i2c(s);
	result |= inv_icm20602_secondary_stop_channel(s, index);
	result |= inv_icm20602_set_chip_power_state(s, CHIP_LP_ENABLE, !!(orig_power_state & CHIP_LP_ENABLE));
	result |= inv_icm20602_set_chip_power_state(s, CHIP_AWAKE, !!(orig_power_state & CHIP_AWAKE));

	return result;
}

int inv_icm20602_secondary_stop_channel(struct inv_icm20602 * s, int index)
{
	uint8_t data = 0;
	
	return inv_icm20602_mems_write_reg(s, s->secondary_state.slv_reg[index].ctrl, 1, &data);
}

int inv_icm20602_secondary_enable_i2c(struct inv_icm20602 * s)
{
	uint8_t data = s->base_state.user_ctrl |= BIT_I2C_MST_EN;;

	return inv_icm20602_mems_write_reg(s, REG_USER_CTRL, 1, &data); 
}

int inv_icm20602_secondary_disable_i2c(struct inv_icm20602 * s)
{
	uint8_t data = s->base_state.user_ctrl &= ~BIT_I2C_MST_EN;
	return inv_icm20602_mems_write_reg(s, REG_USER_CTRL, 1, &data); 
}

#endif
