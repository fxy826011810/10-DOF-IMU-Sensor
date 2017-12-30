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

/** @defgroup DriverIcm20602 Icm20602 driver
 *  @brief    Low-level driver for Icm20602 devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICM20602_H_
#define _INV_ICM20602_H_

#include "Invn/InvBool.h"
#include "Invn/InvError.h"

#include "Invn/Devices/Drivers/Icm20602/Icm20602Ctrl.h"
#include "Invn/Devices/Drivers/Icm20602/Icm20602SelfTest.h"
#include "Invn/Devices/Drivers/Icm20602/Icm20602Serif.h"
#include "Invn/Devices/Drivers/Icm20602/Icm20602Setup.h"
#include "Invn/Devices/Drivers/Icm20602/Icm20602Transport.h"
#include "Invn/Devices/Drivers/Icm20602/Icm20602AuxTransport.h"
#include "Invn/Devices/Drivers/Icm20602/Icm20602AuxCompassAkm.h"

#include <stdint.h>
#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Icm20602 driver states definition
 */
typedef struct inv_icm20602 {
	struct inv_icm20602_serif serif;
	struct inv_icm20602_serif serif_ois;
	inv_bool_t use_serif_ois; /* set to 1 is ois serif has been set */

	/** @brief icm20602 internal state structure*/
	struct inv_icm20602_states {
		uint8_t  wake_state;
		uint8_t  pwr_mgmt_1;
		uint8_t  pwr_mgmt_2;
		uint8_t  lp_config;
		uint8_t  gyro_div;
		uint8_t  user_ctrl;
		uint16_t accel_div;
		uint16_t compass_div;
		uint8_t  gyro_averaging;
		uint8_t  accel_averaging;
		uint8_t  gyro_fullscale;
	    uint8_t  gyro_ois_fullscale;
		uint8_t  accel_fullscale;
	    uint8_t  accel_ois_fullscale;
		uint8_t  gyro_bw;
		uint8_t  accel_bw;
		uint8_t  lp_en_support:1;
		uint8_t  serial_interface;
		uint8_t  timebase_correction_pll;
		uint16_t sample_rate;
		uint8_t  sample_repeat_cnt;
	} base_state;
	uint8_t  power_state;
	uint32_t android_sensor_mask; // a bit mask corresponding to enum ANDROID_SENSORS
	uint16_t requested_odr[INV_ICM20602_SENSOR_MAX];
	uint8_t  dropped_data[INV_ICM20602_SENSOR_MAX];
	uint8_t  selftest_done;
	uint8_t  wom_enabled;
	/* collected bias values (lsb) during self test */
	int gyro_ois_st_bias[3];
	int accel_ois_st_bias[3];
	int gyro_st_bias[3];
	int accel_st_bias[3];
	/* collected bias value to apply in register */
	uint16_t accel_ois_reg_bias[3];
	uint16_t accel_reg_bias[3];
	uint8_t accel_reg_bias_computed;
	/** @brief secondary device support */
	struct inv_icm20602_secondary_states {
		/** @brief secondary register mapping*/
		struct inv_icm20602_secondary_reg {
			uint16_t addr;
			uint16_t reg;
			uint16_t ctrl;
			uint16_t d0;
		} slv_reg[4];
		/* compass support */
		uint8_t compass_sens[3];
		int16_t *st_upper;
		int16_t *st_lower;
		uint8_t compass_id;
		uint8_t compass_i2c_addr;
		uint8_t compass_aux_ch;
		uint8_t secondary_resume_state;
		uint8_t mode_reg_addr;
		uint8_t i2c_mst_dly;
	} secondary_state;
} inv_icm20602_t;

/** @brief Hook for low-level system sleep() function to be implemented by upper layer
 *  @param[in] ms number of millisecond the calling thread should sleep
 */
extern void inv_icm20602_sleep(int ms);

/** @brief Hook for low-level high res system sleep() function to be implemented by upper layer
 *  ~100us resolution is sufficient
 *  @param[in] us number of us the calling thread should sleep
 */
extern void inv_icm20602_sleep_us(int us);

/** @brief Hook for low-level system time() function to be implemented by upper layer
 *  @return monotonic timestamp in us
 */
extern uint64_t inv_icm20602_get_time_us(void);

/** @brief Hook to get interrupt data ready timestamp to be implemented by upper layer
 *  Using this hook in embedded firmware, the timestamping could be done in ISR and allowed a better accuracy than getting the current time in the polling function
 *  But in host application, this function will have the same implementation than get_time_us()  
 *  @return data ready interrupt timestamp in us
 */
extern uint64_t inv_icm20602_get_dataready_interrupt_time_us(void);

/** @brief Reset and initialize driver states
 *  @param[in] s handle to driver states structure
 *  @param[in] serif handle to SERIF object for underlying register access
 */
static inline void inv_icm20602_reset_states(struct inv_icm20602 * s,
		const struct inv_icm20602_serif * serif)
{
	memset(s, 0, sizeof(*s));
	s->serif = *serif;
}

/** @brief Register secondary SERIF object for OIS access
 *  Must be called after inv_icm20602_reset_states()
 *  @param[in] s handle to driver states structure
 *  @param[in] serif handle to SERIF object for underlying register access to OIS
 */
static inline void inv_icm20602_reset_states_serif_ois(struct inv_icm20602 * s,
        const struct inv_icm20602_serif * serif_ois)
{
	if(serif_ois) {
	    s->serif_ois = *serif_ois;
	    s->use_serif_ois = 1;
	}
}
#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM20602_H_ */

/** @} */
