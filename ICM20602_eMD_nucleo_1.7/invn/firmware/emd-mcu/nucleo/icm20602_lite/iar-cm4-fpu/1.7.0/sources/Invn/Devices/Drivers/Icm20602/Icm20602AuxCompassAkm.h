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
/** @defgroup	DriverIcm20602AuxCompassAkm Icm20602 akm compass support
 *  @brief      Low-level Icm20602 aux sensor access
 *  @ingroup 	DriverIcm20602
 *  @{
*/
#ifndef _INV_ICM20602_AUX_COMPASS_AKM_H_
#define _INV_ICM20602_AUX_COMPASS_AKM_H_

#include "Invn/InvExport.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* forward declaration */
struct inv_icm20602;

/** @brief Supported auxiliary compass identifer
 */
enum inv_icm20602_compass_id {
	INV_ICM20602_COMPASS_ID_NONE = 0, /**< no compass */
	INV_ICM20602_COMPASS_ID_AK09911, 	/**< AKM AK09911 */
	INV_ICM20602_COMPASS_ID_AK09912,  /**< AKM AK09912 */
};

/** @brief Register AUX compass
 *
 *  Will only set internal states and won't perform any transaction on the bus.
 *  Must be called before inv_icm20602_initialize().
 *
 *  @param[in]  compass_id 	        Compass ID
 *  @param[in]  compass_i2c_addr 	Compass I2C address
 *  @return     0 on success, negative value on error
 */
void INV_EXPORT inv_icm20602_register_aux_compass(struct inv_icm20602 * s,
		enum inv_icm20602_compass_id compass_id, uint8_t compass_i2c_addr);

/** @brief Return non-zero value is AUX compass was regitered, 0 if not
 *  If compass was registered but setup failed, will also return 0
 */
int INV_EXPORT inv_icm20602_is_compass_registered(struct inv_icm20602 * s);

/** @brief Initializes the compass
 *  @return    0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_setup_compass_akm(struct inv_icm20602 * s);

/** @brief Self test for the compass
 *  @return    0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_check_akm_self_test(struct inv_icm20602 * s);

#if 0 // disabled for now
/** @brief Changes the scale of the compass
 *  @param[in] data  new scale for the compass
 *  @return          0 in case of success, -1 for any error
 */
int inv_icm20602_write_akm_scale(struct inv_icm20602 * s, int data);
#endif

/** @brief Reads the scale of the compass
 *  @param[out] scale    pointer to recuperate the scale
 *  @return 0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_read_akm_scale(struct inv_icm20602 * s, int *scale);

/** @brief Stops the compass
*   @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20602_suspend_akm(struct inv_icm20602 * s);

/** @brief Starts the compass
 *  @return 0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_resume_akm(struct inv_icm20602 * s);

/** @brief Get compass power status
 *  @return 1 in case compass is enabled, 0 if not started
 */
char INV_EXPORT inv_icm20602_compass_getstate(struct inv_icm20602 * s);

/** @brief Parse compass data packet
 *  @param[in] data packet pointer to compass data packet
 *  @param[in] raw_compass pointer to raw compass. big endian 2 bytes for each axis.
 *  @return 0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_get_compass_data(struct inv_icm20602 * s, const unsigned char *packet, unsigned char *raw_compass);

/** @brief Get data packet size according to comass type
 *  @return size in bytes
 */
int INV_EXPORT inv_icm20602_get_compass_bytes(struct inv_icm20602 * s);

#ifdef __cplusplus
}
#endif

#endif // _INV_ICM20602_AUX_COMPASS_AKM_H_

/** @} */
