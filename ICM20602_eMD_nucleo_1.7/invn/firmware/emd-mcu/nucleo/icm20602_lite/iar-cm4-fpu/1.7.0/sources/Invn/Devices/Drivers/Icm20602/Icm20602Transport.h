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

/** @defgroup DriverIcm20602Transport Icm20602 driver transport
 *  @brief    Low-level Icm20602 register access
 *  @ingroup  DriverIcm20602
 *  @{
 */

#ifndef _INV_ICM20602_TRANSPORT_H_
#define _INV_ICM20602_TRANSPORT_H_

#include "Invn/InvExport.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* forward declaration */
struct inv_icm20602;

int INV_EXPORT inv_icm20602_read_reg(struct inv_icm20602 * s, uint8_t reg, uint8_t * buf, uint32_t len);

int INV_EXPORT inv_icm20602_write_reg(struct inv_icm20602 * s, uint8_t reg, const uint8_t * buf, uint32_t len);

int INV_EXPORT inv_icm20602_read_reg_ois(struct inv_icm20602 * s, uint8_t reg,	uint8_t * buf, uint32_t len);

static inline int inv_icm20602_write_reg_one(struct inv_icm20602 * s, uint8_t reg, uint8_t reg_value)
{
	return inv_icm20602_write_reg(s, reg, &reg_value, 1);
}

static inline int inv_icm20602_read_reg_one(struct inv_icm20602 * s, uint8_t reg, uint8_t * reg_value)
{
	return inv_icm20602_read_reg(s, reg, reg_value, 1);
}

static inline int inv_icm20602_set_reg_bits(struct inv_icm20602 * s, uint8_t reg, uint8_t bits_mask)
{
	int rc;
	uint8_t reg_value;

	if((rc = inv_icm20602_read_reg_one(s, reg, &reg_value)) != 0)
		return rc;

	reg_value |= bits_mask;

	if((rc = inv_icm20602_write_reg_one(s, reg, reg_value)) != 0)
		return rc;

	return 0;
}

static inline int inv_icm20602_clear_reg_bits(struct inv_icm20602 * s, uint8_t reg, uint8_t bits_mask)
{
	int rc;
	uint8_t reg_value;

	if((rc = inv_icm20602_read_reg_one(s, reg, &reg_value)) != 0)
		return rc;

	reg_value &= ~bits_mask;

	if((rc = inv_icm20602_write_reg_one(s, reg, reg_value)) != 0)
		return rc;

	return 0;
}

static inline int inv_icm20602_get_reg_bits(struct inv_icm20602 * s, uint8_t reg,
		uint8_t bits_mask, uint8_t * bits_mask_state)
{
	int rc;

	if((rc = inv_icm20602_read_reg_one(s, reg, bits_mask_state)) != 0)
		return rc;

	*bits_mask_state &= bits_mask;

	return 0;
}

/** @brief Writes data from a register on mems.
 * @param[in]  length number of byte to be written
 * @param[out] data   output data from the register
 * @return            0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_mems_write_reg(struct inv_icm20602 * s, uint16_t reg, uint32_t length, const uint8_t *data);

/** @brief Writes a single byte of data from a register on mems with no power control.
 * @param[in]  reg  Register address
 * @param[out] data	Data to be written
 * @return          0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_mems_write_reg_one(struct inv_icm20602 * s, uint16_t reg, uint8_t data);

/** @brief Reads data from a register on mems.
 * @param[in]   reg    register address
 * @param[in]  	length length of data
 * @param[out]  data   output data from the register
 * @return      0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_mems_read_reg(struct inv_icm20602 * s, uint16_t reg, uint32_t length, uint8_t *data);


#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM20602_TRANSPORT_H_ */

/** @} */
