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

int inv_icm20602_read_reg(struct inv_icm20602 * s, uint8_t reg,	uint8_t * buf, uint32_t len)
{
	return inv_icm20602_serif_read_reg(&s->serif, reg, buf, len);
}

int inv_icm20602_write_reg(struct inv_icm20602 * s, uint8_t reg, const uint8_t * buf, uint32_t len)
{
	return inv_icm20602_serif_write_reg(&s->serif, reg, buf, len);
}

int inv_icm20602_read_reg_ois(struct inv_icm20602 * s, uint8_t reg,	uint8_t * buf, uint32_t len)
{
	return inv_icm20602_serif_read_reg(&s->serif_ois, reg, buf, len);
}

// Max size that can be read across I2C or SPI data lines
#define MAX_SERIAL_READ 16
// Max size that can be written across I2C or SPI data lines
#define MAX_SERIAL_WRITE 16

// These regsiters are in sclk or sclk/mclk domains and
// can be written in LP mode or sleep state
static uint8_t check_reg_access_lp_disable(unsigned short reg)
{
#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
	(void)reg;
	// All Athens registers are in sclk domain
	return 0;
#else
	switch(reg){
		case 0x19:
		case 0x1D:
		case 0x1E:
		case 0x37:
		case 0x38:
		case 0x6B:
		case 0x6C:
		case 0x72:
		case 0x73:
		case 0x74:
			return 0;
		default:
			break;
	}
	return 1;
#endif
}


/* the following functions are used for configuring the secondary devices */

int inv_icm20602_mems_write_reg(struct inv_icm20602 * s, uint16_t reg, uint32_t length, const uint8_t *data)
{
    int result = 0;
	uint32_t bytesWrite = 0;
    const uint8_t regOnly = (uint8_t)(reg & 0x7F);

    const uint8_t orig_power_state = inv_icm20602_get_chip_power_state(s);

	if((orig_power_state & CHIP_AWAKE) == 0) {
		// Wake up chip since it is asleep
		if(check_reg_access_lp_disable(reg))    // Check if register needs to be awake
			result = inv_icm20602_set_chip_power_state(s, CHIP_AWAKE, 1);
	}

	if((orig_power_state & CHIP_LP_ENABLE) != 0) {
		if(check_reg_access_lp_disable(reg))    // Check if register needs LP_EN to be disabled
			result |= inv_icm20602_set_chip_power_state(s, CHIP_LP_ENABLE, 0);   // Disable LP_EN
	}

	while (bytesWrite<length)
	{
		int thisLen = INV_MIN(MAX_SERIAL_WRITE, length-bytesWrite);

		result |= inv_icm20602_write_reg(s, regOnly+bytesWrite, &data[bytesWrite], thisLen);

		if (result)
			return result;

		bytesWrite += thisLen;
	}

    // Put the chip back to LP mode if it was enabled originally
	if((orig_power_state & CHIP_LP_ENABLE) != 0) {
		if(check_reg_access_lp_disable(reg))    // Check if register needs LP_EN to be disabled
			result |= inv_icm20602_set_chip_power_state(s, CHIP_LP_ENABLE, 1);   // Enable LP_EN
	}

    // Put the chip back to sleep if it was asleep originally
	if((orig_power_state & CHIP_AWAKE) == 0) {
		if(check_reg_access_lp_disable(reg))    // Check if register needs to be awake
			result |= inv_icm20602_set_chip_power_state(s, CHIP_AWAKE, 0);
	}

	return result;
}

int inv_icm20602_mems_write_reg_one(struct inv_icm20602 * s, uint16_t reg, uint8_t data)
{
	const uint8_t regOnly = (uint8_t)(reg & 0x7F);

	return inv_icm20602_write_reg(s, regOnly, &data, 1);
}

int inv_icm20602_mems_read_reg(struct inv_icm20602 * s, uint16_t reg, uint32_t length, uint8_t *data)
{
	int result = 0;
	uint32_t bytesRead = 0;
	const uint8_t regOnly = (uint8_t)(reg & 0x7F);

	while(bytesRead<length) {
		int thisLen = INV_MIN(MAX_SERIAL_READ, length-bytesRead);

		result |= inv_icm20602_read_reg(s, regOnly+bytesRead, &data[bytesRead], thisLen);

		if (result)
		return result;

		bytesRead += thisLen;
	}

	return result;
}
