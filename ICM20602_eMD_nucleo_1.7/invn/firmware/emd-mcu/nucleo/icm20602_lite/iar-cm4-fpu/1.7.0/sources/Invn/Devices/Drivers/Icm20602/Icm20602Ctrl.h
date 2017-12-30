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

/** @defgroup DriverIcm20602Ctrl Icm20602 control
 *  @brief Low-level function to control a Icm20602 device
 *  @ingroup  DriverIcm20602
 *  @{
 */

#ifndef _INV_ICM20602_CTRL_H_
#define _INV_ICM20602_CTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Invn/InvExport.h"

#include <stdint.h>

/* forward declaration */
struct inv_icm20602;

/** @brief Sensor identifier for control function
 */
enum inv_icm20602_sensor {
	INV_ICM20602_SENSOR_ACCEL,
	INV_ICM20602_SENSOR_GYRO,
	INV_ICM20602_SENSOR_COMPASS,
	INV_ICM20602_SENSOR_FSYNC_EVENT,
	INV_ICM20602_SENSOR_OIS,
	INV_ICM20602_SENSOR_TEMPERATURE,
	INV_ICM20602_SENSOR_MAX
};

/** @brief Enables accel and/or gyro and/or pressure if integrated with gyro and accel.
 *  @param[in] bit_mask A mask where 2 means turn on accel, 1 means turn on gyro, 4 is for pressure.
 *            			By default, this only turns on a sensor if all sensors are off otherwise the DMP controls
 *                      this register including turning off a sensor. To override this behavior add in a mask of 128.
 *  @param[in] smplrt_divider  The divider which was applied to internal sample rate based on field sample_rate
 *                            from base_driver_t to get minimum ODR for accel and gyro
 *  @return 0 on success, negative value on error
 */
int INV_EXPORT inv_icm20602_enable_mems(struct inv_icm20602 * s, int bit_mask, uint16_t smplrt_divider);

/** @brief Sets the odr for a sensor
 *  @param[in] sensor  Sensor Identity
 *  @param[in] delayInMs  the delay between two values in ms
 *  @return 0 in case of success, -1 for any error
 */
int INV_EXPORT inv_icm20602_set_sensor_period(struct inv_icm20602 * s, enum inv_icm20602_sensor sensor, uint16_t delayInMs);

/** @brief Knows if the sensor is enabled or disbaled
 *  @param[in] sensor  the sensor is enabled or not
 *  @return 1 if active, 0 otherwise
 */
int INV_EXPORT inv_icm20602_is_sensor_enabled(struct inv_icm20602 * s, enum inv_icm20602_sensor sensor);

/** @brief Enables / disables a sensor
 * @param[in] androidSensor  Sensor Identity
 * @param[in] enable			0=off, 1=on
 * @return 0 in case of success, negative value on error
 */
int INV_EXPORT inv_icm20602_enable_sensor(struct inv_icm20602 * s, enum inv_icm20602_sensor sensor, uint8_t enable);

/** @brief  Configuress accel WOM.
 *  @param[in] wom_threshold threshold value for X,Y,Z axis that should trigger a WOM interrupt (0 to diable WOM)
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20602_configure_accel_wom(struct inv_icm20602 * s, uint8_t wom_threshold);

/** @brief Check and retrieve for new data
 *  @param[out] acc_data raw gyro data
 *  @param[out] temp_data raw temp data
 *  @param[out] gyro_data raw gyro data
 *  @return 0    : not data ready
 *          0x01 : gyro data output set
 *          0x02 : acc data output set
 *          0x03 : gyro and acc data output set
 *          < 0  : error
 */
int INV_EXPORT inv_icm20602_poll_sensor_data_reg(struct inv_icm20602 * s, 
		int16_t acc_data[3], int16_t *temp_data, int16_t gyro_data[3]);

/** @brief Check and retrieve for new data
 */
struct inv_icm20602_fifo_states {
	uint8_t overflow:1; /**< indicates FIFO overflow: user should restart all sensors if this occurs */
	uint16_t sensor_on_mask;
	uint16_t packet_count_i;
	uint16_t packet_count;
	uint16_t packet_size;
};

/** @brief Initializes states for FIFO reading and parsing
 *  @param[in] states     placeholder to FIFO states
 *  @param[in] int_status int status register value (used to check for overflow)
 *  @return    0 for success, 1 for overlfow detected, negative value for other errors
 */
int INV_EXPORT inv_icm20602_poll_fifo_data_setup(struct inv_icm20602 * s,
		struct inv_icm20602_fifo_states * states, uint8_t int_status);

/** @brief Read one packet from the FIFO (packet corresponding to data for all current active sensor)
 *  @param[in] states     placeholder to FIFO states
 *  @param[out] acc_data raw gyro data
 *  @param[out] temp_data raw temp data
 *  @param[out] gyro_data raw gyro data
 *  @param[out] compass_data raw compass data
 *  @return 0    : FIFO empty
 *          bit0 : gyro data output set
 *          bit1 : acc data output set
 *          bit2 : compass output set
 *          < 0  : error
  */
int INV_EXPORT inv_icm20602_poll_fifo_data(struct inv_icm20602 * s, struct inv_icm20602_fifo_states * states, 
		int16_t acc_data[3], int16_t *temp_data, int16_t gyro_data[3], int16_t compass_data[3]);

/** @brief Read interrupt status and check for DDRY flag
 *  @return 1 if DDRY inteript is set, 0 otherwise and negative value on error
 */
int INV_EXPORT inv_icm20602_has_data_ready(struct inv_icm20602 * s);

/** @brief Read interrupt status and return its value
 *  @param[out] int_status INT status register value
 *  @return 0 on success, negative value on error
 */
int INV_EXPORT inv_icm20602_get_int_status(struct inv_icm20602 * s, uint8_t * int_status);

/** @brief Check for DRDY flag in INT register value
 *  @param[in] int_status INT status register value
 *  @return 1 if DDRDY is set, 0 otherwise
 */
int INV_EXPORT inv_icm20602_check_drdy(struct inv_icm20602 * s, uint8_t int_status);

/** @brief Check for WOM bits in INT register value
 *  @param[in] int_status INT status register value
 *  @return bit mask corresponding to x, y, z axis that caused WOM
 *          (0x01: x axis, 0x02: y axis, 0x01: z axis)
 */
int INV_EXPORT inv_icm20602_check_wom_status(struct inv_icm20602 * s, uint8_t int_status);

/** @brief Disable FIFO
 * @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20602_disable_fifo(struct inv_icm20602 * s);

/** @brief    Enable FIFO
 * @param[in] bit_mask 	A mask of sensor to push to FIFO.
 * @return    0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20602_enable_fifo(struct inv_icm20602 * s, int bit_mask);

/** @brief  Reset FIFO
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20602_reset_fifo(struct inv_icm20602 * s);

/** @brief  Initalize AUX I2C and Compass
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20602_set_slave_compass_id(struct inv_icm20602 * s);

/** @brief  Poll OIS gyro data
 *  @return 0x10 if OIS data reported, 0 otherwise, negative value on error.
 */
int INV_EXPORT inv_icm20602_poll_ois_gyro_data(struct inv_icm20602 * s, int16_t gyro_data[3]);

/** @brief  Poll EIS flag and delay counter
 *  @param[out] delay_count delay time in us between the FSYNC event (before the gyro data event) and the gyro data event
 *  @return 0x8 if FSYNC event reported, 0 otherwise, negative value on error.
 */
int INV_EXPORT inv_icm20602_poll_delay_count(struct inv_icm20602 * s, int16_t * delay_count);

/** @brief  test if all sensors are off
 *  @return true if all sensors are off, false otherwise
 */
int INV_EXPORT inv_icm20602_all_sensors_off(struct inv_icm20602 * s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM20602_CTRL_H_ */

/** @} */
