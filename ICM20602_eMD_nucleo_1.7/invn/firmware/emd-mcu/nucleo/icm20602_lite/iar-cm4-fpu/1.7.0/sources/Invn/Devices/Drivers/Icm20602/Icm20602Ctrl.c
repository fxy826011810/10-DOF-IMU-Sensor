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

#include "Icm20602Ctrl.h"
#include "Icm20602Defs.h"
#include "Icm20602.h"

#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/EmbUtils/Message.h"

// Determines which base sensor needs to be on based upon s->android_sensor_mask[0]
#define INV_NEEDS_ACCEL_MASK    (1L<<ANDROID_SENSOR_ACCELEROMETER)
#define INV_NEEDS_GYRO_MASK     (1L<<ANDROID_SENSOR_GYROSCOPE)
#define INV_NEEDS_COMPASS_MASK  (1L<<ANDROID_SENSOR_MAGNETOMETER)
#define INV_NEEDS_OIS_MASK      (1L<<ANDROID_SENSOR_OIS)
#define INV_NEEDS_TEMP_MASK     (1L<<ANDROID_SENSOR_TEMPERATURE)
#define INV_NEEDS_FSYNC_MASK    (1L<<ANDROID_SENSOR_FSYNC_EVENT)

#define GYRO_AVAILABLE      0x01
#define ACCEL_AVAILABLE     0x02
#define OIS_AVAILABLE       0x04
#define SECONDARY_AVAILABLE 0x08
#define HIGH_ODR_REQUESTED  0x10
#define TEMP_AVAILABLE      0x20
#define COMPASS_AVAILABLE   0x40


/* Convert public sensor type for Icm20602 to internal sensor id (Android)
 */
static uint8_t sensor_type_2_android_sensor(enum inv_icm20602_sensor sensor)
{
	switch(sensor) {
	case INV_ICM20602_SENSOR_ACCEL:       return ANDROID_SENSOR_ACCELEROMETER;
	case INV_ICM20602_SENSOR_GYRO:        return ANDROID_SENSOR_GYROSCOPE;
	case INV_ICM20602_SENSOR_FSYNC_EVENT: return ANDROID_SENSOR_FSYNC_EVENT;
	case INV_ICM20602_SENSOR_OIS:         return ANDROID_SENSOR_OIS;
	case INV_ICM20602_SENSOR_TEMPERATURE: return ANDROID_SENSOR_TEMPERATURE;
	case INV_ICM20602_SENSOR_COMPASS:     return ANDROID_SENSOR_MAGNETOMETER;
	default:                                    return ANDROID_SENSOR_NUM_MAX;
	}
}

static uint32_t is_android_sensor_enabled(struct inv_icm20602 * s, uint8_t androidSensor)
{
	return (s->android_sensor_mask & (1L << (androidSensor & 0x1F)));
}

static int apply_accel_fsr(struct inv_icm20602 * s)
{
	int result = 0;
	uint8_t accel_config_1_reg;

	// Set FSR
	result |= inv_icm20602_mems_read_reg(s, MPUREG_ACCEL_CONFIG, 1, &accel_config_1_reg);
	accel_config_1_reg &= ~BITS_ACCEL_FS_SEL;//0xE7;
	accel_config_1_reg |= (s->base_state.accel_fullscale << BIT_POS_ACCEL_FS_SEL);
	result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_CONFIG, 1, &accel_config_1_reg);

	return result;
}

static int apply_accel_bw(struct inv_icm20602 * s)
{
	int result = 0;
	uint8_t accel_config_2_reg;
	uint8_t dec2_cfg;

	// Set averaging
	switch(s->base_state.accel_averaging) {
	case 1:  dec2_cfg = 0; break;
	case 4:  dec2_cfg = 0; break;
	case 8:  dec2_cfg = 1; break;
	case 16: dec2_cfg = 2; break;
	case 32: dec2_cfg = 3; break;
	default: dec2_cfg = 0; break;
	}

	result |= inv_icm20602_mems_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &accel_config_2_reg);
	accel_config_2_reg &= ~BITS_DEC2_CFG;
	accel_config_2_reg &= ~BITS_A_DLPF_CFG;

	// Set DEC2 and A_DLPF_CFG
	if(inv_icm20602_get_chip_power_state(s) & CHIP_LP_ENABLE) // accel LP mode
	{
		accel_config_2_reg |= 7; // A_DLPF_CFG = 7 
		accel_config_2_reg |= (dec2_cfg << BIT_POS_DEC2_CFG);
	}
	else // accel LN mode
	{
		if(s->base_state.accel_bw > 6)
			accel_config_2_reg |= MPU_ABW_218; // 0 < A_DLPF_CFG < 7
		else
			accel_config_2_reg |= s->base_state.accel_bw;
		dec2_cfg = 0;		
		accel_config_2_reg |= (dec2_cfg << BIT_POS_DEC2_CFG);
	}
	result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &accel_config_2_reg);

	return result;
}

static int apply_accel_bias(struct inv_icm20602 * s)
{
	int i, axis_sign;
	int rc = -1;
	uint16_t accel_reg_otp[3], sensor_bias[3];
	uint8_t axis, d[2];

	// test if self-test bias has been computed
	if (!s->selftest_done)
		return 0;

	if(!s->accel_reg_bias_computed) {
		// read accelerometer bias registers
		rc = inv_icm20602_mems_read_reg(s, MPUREG_XA_OFFS_H, 2, d);
		accel_reg_otp[0] = ((uint16_t)d[0] << 8) | d[1];
		rc = inv_icm20602_mems_read_reg(s, MPUREG_YA_OFFS_H, 2, d);
		accel_reg_otp[1] = ((uint16_t)d[0] << 8) | d[1];
		rc = inv_icm20602_mems_read_reg(s, MPUREG_ZA_OFFS_H, 2, d);
		accel_reg_otp[2] = ((uint16_t)d[0] << 8) | d[1];

		// determine gravity axis (by default on LN values)
		axis = 0;
		axis_sign = 1;
		if (INV_ABS(s->accel_ois_st_bias[1]) > INV_ABS(s->accel_ois_st_bias[0]))
			axis = 1;
		if (INV_ABS(s->accel_ois_st_bias[2]) > INV_ABS(s->accel_ois_st_bias[axis]))
			axis = 2;
		if(s->accel_ois_st_bias[axis] < 0)
			axis_sign = -1;

		// compute register bias value according to power mode (LP/LN)
		for(i = 0; i < 3; i++) {
			// convert from 2g to 16g (1LSB = 0.49mg) on 15bits register and mask bit0
			// note : there is no need to shift the register value as the register is 0.98mg steps
			s->accel_reg_bias[i] = ((uint16_t)(accel_reg_otp[i] - (s->accel_st_bias[i] / 8)) & ~1);
			s->accel_ois_reg_bias[i] = ((uint16_t)(accel_reg_otp[i] - (s->accel_ois_st_bias[i] / 8)) & ~1);
			// remove gravity for axis
			if(i == axis) {
				s->accel_reg_bias[i] += ((DEF_ST_SCALE / DEF_ST_ACCEL_FS) / 8) * axis_sign; 
				s->accel_ois_reg_bias[i] += ((DEF_ST_SCALE / DEF_ST_ACCEL_FS) / 8) * axis_sign; 
			}
		}
		s->accel_reg_bias_computed = 1;
	}

	//get bias value according to power mode
	if((s->power_state == PowerStateAccLPState) || (s->power_state == PowerState6AxisLPState)) {
		for(i = 0; i < 3; i++)
			sensor_bias[i] = s->accel_reg_bias[i];
	} else if((s->power_state == PowerStateAccLNState) || (s->power_state == PowerState6AxisLNState)) {
		for(i = 0; i < 3; i++)
			sensor_bias[i] = s->accel_ois_reg_bias[i];
	} else
		return rc;

	d[0] = (sensor_bias[0] >> 8) & 0xff;
	d[1] = (sensor_bias[0] & 0xff);
	rc = inv_icm20602_mems_write_reg(s, MPUREG_XA_OFFS_H, 2, d);

	d[0] = (sensor_bias[1] >> 8) & 0xff;
	d[1] = sensor_bias[1] & 0xff;
	rc |= inv_icm20602_mems_write_reg(s, MPUREG_YA_OFFS_H, 2, d);

	d[0] = (sensor_bias[2] >> 8) & 0xff;
	d[1] = sensor_bias[2] & 0xff;
	rc |= inv_icm20602_mems_write_reg(s, MPUREG_ZA_OFFS_H, 2, d);

	return rc;
}

static int apply_gyr_fsr_bw(struct inv_icm20602 * s)
{
	int result = 0;
	uint8_t data;
	uint8_t gyro_config_1_reg;
	uint8_t g_avgcfg;

	// Set FSR
	result |= inv_icm20602_mems_read_reg(s, MPUREG_GYRO_CONFIG, 1, &gyro_config_1_reg);
	gyro_config_1_reg &= ~BITS_GYRO_FS_SEL;
	gyro_config_1_reg |= (inv_icm20602_get_gyro_fullscale(s)<< BIT_POS_GYRO_FS_SEL);
	result |= inv_icm20602_mems_write_reg(s, MPUREG_GYRO_CONFIG, 1, &gyro_config_1_reg);

	// Set averaging
	switch(s->base_state.gyro_averaging) {
	case 1:   g_avgcfg = 0; break;
	case 2:   g_avgcfg = 1; break;
	case 4:   g_avgcfg = 2; break;
	case 8:   g_avgcfg = 3; break;
	case 16:  g_avgcfg = 4; break;
	case 32:  g_avgcfg = 5; break;
	case 64:  g_avgcfg = 6; break;
	case 128: g_avgcfg = 7; break;
	default:  g_avgcfg = 0; break;
	}

	result |= inv_icm20602_mems_read_reg(s, MPUREG_LP_CONFIG, 1, &data);
	data &= ~BITS_G_AVGCFG;
	data |= (g_avgcfg<<BIT_POS_G_AVGCFG);
	result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &data);

	// Set DLPF_CFG
	result |= inv_icm20602_mems_read_reg(s, MPUREG_CONFIG, 1, &data);
	data &= ~BITS_DLPF_CFG; // Clear bits[2:0]
	data |= inv_icm20602_get_gyro_bandwidth(s);
	result |= inv_icm20602_mems_write_reg(s, MPUREG_CONFIG, 1, &data);

	return result;
}

static int apply_gyr_bias(struct inv_icm20602 * s)
{
	int i;
	int rc = -1;
	int sensor_bias[3];
	uint8_t data[2];

	// test if self-test bias has been computed
	if (!s->selftest_done)
			return 0;

	for(i = 0; i < 3; i++) {

		// get bias value according to power mode
		if((s->power_state == PowerStateGyrLPState) || (s->power_state == PowerState6AxisLPState))
			sensor_bias[i] = s->gyro_st_bias[i];
		else if((s->power_state == PowerStateGyrLNState) || (s->power_state == PowerState6AxisLNState))
			sensor_bias[i] = s->gyro_ois_st_bias[i];
		else
			return rc;

		// convert bias value from 250dps to 1000dps
		sensor_bias[i] = - sensor_bias[i] / 4;
	}

	data[0] = (sensor_bias[0] >> 8) & 0xff;
	data[1] = sensor_bias[0] & 0xff;
	rc = inv_icm20602_mems_write_reg(s, MPUREG_XG_OFFS_USRH, 2, data);

	data[0] = (sensor_bias[1] >> 8) & 0xff;
	data[1] = sensor_bias[1] & 0xff;
	rc |= inv_icm20602_mems_write_reg(s, MPUREG_YG_OFFS_USRH, 2, data);

	data[0] = (sensor_bias[2] >> 8) & 0xff;
	data[1] = sensor_bias[2] & 0xff;
	rc |= inv_icm20602_mems_write_reg(s, MPUREG_ZG_OFFS_USRH, 2, data);

	return rc;
}

static int enable_interrupt(struct inv_icm20602 * s, int bit_mask)
{
	int result = 0;
	uint8_t data;

	result |= inv_icm20602_mems_read_reg(s, MPUREG_INT_ENABLE, 1, &data);

	if ((bit_mask == OIS_AVAILABLE) || !bit_mask)
		data &= ~BIT_DATA_RDY_INT_EN; // clear data ready interrupt
	else
		data |= BIT_DATA_RDY_INT_EN; // Set data ready interrupt

	result |= inv_icm20602_mems_write_reg(s, MPUREG_INT_ENABLE, 1, &data);

	return result;
}

int inv_icm20602_has_data_ready(struct inv_icm20602 * s)
{
	uint8_t int_status;

	const int rc = inv_icm20602_read_reg_one(s, REG_INT_STATUS, &int_status);

	if(rc == 0 && (int_status & BIT_DATA_RDY_INT)) {
		return 1;
	}

	return rc;
}

int inv_icm20602_get_int_status(struct inv_icm20602 * s, uint8_t * int_status)
{
	return inv_icm20602_read_reg_one(s, REG_INT_STATUS, int_status);
}

int inv_icm20602_check_drdy(struct inv_icm20602 * s, uint8_t int_status)
{
	(void)s;

	if(int_status & BIT_DATA_RDY_INT)
		return 1;

	return 0;
}

int inv_icm20602_check_wom_status(struct inv_icm20602 * s, uint8_t int_status)
{
	int wom_data = 0;

	(void)s;

	if(int_status & BIT_WOM_X_INT)
		wom_data |= 0x1;
	if(int_status & BIT_WOM_Y_INT)
		wom_data |= 0x2;
	if(int_status & BIT_WOM_Z_INT)
		wom_data |= 0x4;

	return wom_data;
}

int inv_icm20602_poll_sensor_data_reg(struct inv_icm20602 * s, int16_t acc_data[3], int16_t * temp_data, int16_t gyro_data[3])
{
	int rc;
	uint8_t data[14];
	uint8_t  reg    = MPUREG_ACCEL_XOUT_H;
	uint8_t  len    = 14; /* includes temp reg */
	uint8_t  offset = 0;

	const int acc_on = inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_ACCEL);
	const int gyro_on = inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_GYRO);
	const int temp_on = inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_TEMPERATURE);

	if(acc_on && !gyro_on) {
		reg    = MPUREG_ACCEL_XOUT_H;
		len    = 6;
		offset = 0;
		if(temp_on)
			len = 8;
	}
	else if(!acc_on && gyro_on) {
		if(temp_on) {
			reg    = MPUREG_TEMP_XOUT_H;
			len    = 8;
			offset = 6;
		} else {
			reg    = MPUREG_GYRO_XOUT_H;
			len    = 6;
			offset = 8;
		}
	}

	if((rc = inv_icm20602_read_reg(s, reg, &data[offset], len)) != 0)
		return rc;

	if(acc_on) {

		/* raw sensor data output is in big endian */
		acc_data[0] = (data[0] << 8) | data[1];
		acc_data[1] = (data[2] << 8) | data[3];
		acc_data[2] = (data[4] << 8) | data[5];

		rc |= (1 << INV_ICM20602_SENSOR_ACCEL);
	}

	if(temp_on) {

		*temp_data = (data[0+6] << 8) | data[1+6];

		rc |= (1 << INV_ICM20602_SENSOR_TEMPERATURE);
	}

	if(gyro_on) {

		gyro_data[0] = (data[0+8] << 8) | data[1+8];
		gyro_data[1] = (data[2+8] << 8) | data[3+8];
		gyro_data[2] = (data[4+8] << 8) | data[5+8];

		rc |= (1 << INV_ICM20602_SENSOR_GYRO);
	}

	return rc;
}

int inv_icm20602_poll_fifo_data_setup(struct inv_icm20602 * s,
		struct inv_icm20602_fifo_states * states, uint8_t int_status)
{
#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20603)||(MEMS_CHIP == HW_ICM20789)||(MEMS_CHIP == HW_ICM20609))
	int rc;
	uint8_t data[2];

	states->sensor_on_mask = 0;
	states->overflow       = 0;
	states->packet_count_i = 0;
	states->packet_count   = 0;
	states->packet_size    = 0;

	if(inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_ACCEL)) {
		states->sensor_on_mask |= (1 << INV_ICM20602_SENSOR_ACCEL);
		states->packet_size += ACCEL_DATA_SIZE;
	}

	if(inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_GYRO)) {
		states->sensor_on_mask |= (1 << INV_ICM20602_SENSOR_GYRO);
		states->packet_size += GYRO_DATA_SIZE;
	}
	
#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
	if(inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_ACCEL) ||
	   inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_GYRO) ) {
		states->packet_size += TEMP_DATA_SIZE;
	}
	if(inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_TEMPERATURE)) {
		states->sensor_on_mask |= (1 << INV_ICM20602_SENSOR_TEMPERATURE);
	}
#endif

#if ((MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20789)||(MEMS_CHIP == HW_ICM20609))
	if(inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_TEMPERATURE)) {
		states->sensor_on_mask |= (1 << INV_ICM20602_SENSOR_TEMPERATURE);
		states->packet_size += TEMP_DATA_SIZE;
	}
#endif
#if (MEMS_CHIP == HW_ICM20690)
	if(inv_icm20602_is_sensor_enabled(s, INV_ICM20602_SENSOR_COMPASS)) {
		states->sensor_on_mask |= (1 << INV_ICM20602_SENSOR_COMPASS);
		states->packet_size += inv_icm20602_get_compass_bytes(s);
	}
#endif

	if(int_status & BIT_FIFO_OVERFLOW) {
		uint16_t i, sensor_on_mask = states->sensor_on_mask;
		states->overflow = 1;
		/* restart all sensors active sensor */
		for(i = 0; sensor_on_mask != 0 ; ++i) {
			if(sensor_on_mask & 1) {
				inv_icm20602_enable_sensor(s, (enum inv_icm20602_sensor)i, 0);
				inv_icm20602_enable_sensor(s, (enum inv_icm20602_sensor)i, 1);
			}
			sensor_on_mask >>= 1;
		}

		return 1;
	}

	if((rc = inv_icm20602_read_reg(s, MPUREG_FIFO_COUNTH, data, 2)) != 0)
		return rc;

	states->packet_count = (data[0] << 8) | data[1]; /* FIFO record mode */

	return 0;
#else
	/* For 20602, do some dummy setup to fake FIFO reading */

	states->sensor_on_mask = 0;
	states->overflow       = 0;
	states->packet_count_i = 0;
	states->packet_size    = 0;
	states->packet_count   = 0;

	if(int_status & BIT_DATA_RDY_INT) {
		states->packet_count  = 1;
	}

	(void)s;

	return 0;
#endif
}


int inv_icm20602_poll_fifo_data(struct inv_icm20602 * s,
		struct inv_icm20602_fifo_states * states, int16_t acc_data[3],
		int16_t * temp_data, int16_t gyro_data[3], int16_t compass_data[3])
{
#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20603)||(MEMS_CHIP == HW_ICM20789)||(MEMS_CHIP == HW_ICM20609))
	if(states->packet_count_i < states->packet_count) {
		static int repeated_mems_data = 0;
		int rc;
		uint8_t data[ACCEL_DATA_SIZE + TEMP_DATA_SIZE + GYRO_DATA_SIZE + COMPASS_MAX_DATA_SIZE];
		uint8_t idx = 0;
#if (MEMS_CHIP == HW_ICM20690)
		static int repeated_compass_data = 0;
		uint8_t d[COMPASS_MAX_DATA_SIZE];
		uint8_t *p;
#else
		(void)compass_data;
#endif

		if((rc = inv_icm20602_read_reg(s, MPUREG_FIFO_R_W, data, states->packet_size)) != 0) {
			/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
				   reset FIFO and try next chance */
			inv_icm20602_reset_fifo(s);
			return rc;
		}

		if(states->sensor_on_mask & (1 << INV_ICM20602_SENSOR_ACCEL)) {
			if(!s->dropped_data[INV_ICM20602_SENSOR_ACCEL]) {
				if(!repeated_mems_data) {
					/* raw sensor data output is in big endian */
					acc_data[0] = (data[0] << 8) | data[1];
					acc_data[1] = (data[2] << 8) | data[3];
					acc_data[2] = (data[4] << 8) | data[5];

					rc |= (1 << INV_ICM20602_SENSOR_ACCEL);
				}
			}
			else 
				s->dropped_data[INV_ICM20602_SENSOR_ACCEL]--;

			idx += ACCEL_DATA_SIZE;
		}

#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20603))
		if(states->sensor_on_mask) {
			/* raw sensor data output is in big endian */
			*temp_data = (data[0+idx] << 8) | data[1+idx];

			// Only notify caller that the temperature is available if temperature is acutally enabled.
			if(states->sensor_on_mask & (1 << INV_ICM20602_SENSOR_TEMPERATURE)) {
				if(!s->dropped_data[INV_ICM20602_SENSOR_TEMPERATURE])
					rc |= (1 << INV_ICM20602_SENSOR_TEMPERATURE);
				else
					s->dropped_data[INV_ICM20602_SENSOR_TEMPERATURE]--;
			}
			idx += TEMP_DATA_SIZE;
		}
#endif

#if ((MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20789)||(MEMS_CHIP == HW_ICM20609))
		if(states->sensor_on_mask & (1 << INV_ICM20602_SENSOR_TEMPERATURE)) {
			if(!repeated_mems_data) {
				if(!s->dropped_data[INV_ICM20602_SENSOR_TEMPERATURE]) {
					/* raw sensor data output is in big endian */
					*temp_data = (data[0+idx] << 8) | data[1+idx];

					rc |= (1 << INV_ICM20602_SENSOR_TEMPERATURE);
				} else {
					s->dropped_data[INV_ICM20602_SENSOR_TEMPERATURE]--;
				}
			}
			idx += TEMP_DATA_SIZE;
		}
#endif

		if(states->sensor_on_mask & (1 << INV_ICM20602_SENSOR_GYRO)) {
			if(!s->dropped_data[INV_ICM20602_SENSOR_GYRO]) {
				if(!repeated_mems_data) {
					gyro_data[0] = (data[0+idx] << 8) | data[1+idx];
					gyro_data[1] = (data[2+idx] << 8) | data[3+idx];
					gyro_data[2] = (data[4+idx] << 8) | data[5+idx];

					rc |= (1 << INV_ICM20602_SENSOR_GYRO);
				}
			}
			else
				s->dropped_data[INV_ICM20602_SENSOR_GYRO]--;

			idx += GYRO_DATA_SIZE;
		}
		
#if (MEMS_CHIP == HW_ICM20690)
		if(states->sensor_on_mask & (1 << INV_ICM20602_SENSOR_COMPASS)) {
			if(!repeated_compass_data) {
				p = &data[idx];
				inv_icm20602_get_compass_data(s, p, d);
				compass_data[0] = (d[0] << 8) | d[1];
				compass_data[1] = (d[2] << 8) | d[3];
				compass_data[2] = (d[4] << 8) | d[5];

				rc |= (1 << INV_ICM20602_SENSOR_COMPASS);
			}
			repeated_compass_data = (repeated_compass_data+1) % (s->secondary_state.i2c_mst_dly+1);
			idx += inv_icm20602_get_compass_bytes(s);
		}
#endif

		repeated_mems_data = (repeated_mems_data+1) % (s->base_state.sample_repeat_cnt);
		states->packet_count_i += 1;

		return rc;
	}
	else {
		return 0;
	}
#else
	(void)compass_data;

	/* just read normal register */
	if(states->packet_count_i < states->packet_count) {
		states->packet_count_i += 1;
		return inv_icm20602_poll_sensor_data_reg(s, acc_data, temp_data, gyro_data);
	}
	else {
		return 0;
	}
#endif
}

int inv_icm20602_enable_mems(struct inv_icm20602 * s, int bit_mask, uint16_t smplrt_divider)
{
	int result = 0;

#if (MEMS_CHIP == HW_ICM20690)
	// Configure ois_enable bit
	uint8_t data = 0;
	uint8_t fs_sel = 0;
	result |= inv_icm20602_mems_read_reg(s, MPUREG_USER_CTRL_NEW, 1, &data);

	if(bit_mask & OIS_AVAILABLE) {
		// Set FSR
		result |= inv_icm20602_mems_read_reg(s, MPUREG_ACCEL_CONFIG, 1, &fs_sel);
		fs_sel &= ~BITS_ACCEL_FS_SEL_OIS;
		fs_sel |= (s->base_state.accel_ois_fullscale);
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_CONFIG, 1, &fs_sel);

		result |= inv_icm20602_mems_read_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &fs_sel);
		fs_sel &= ~BITS_GYRO_FS_SEL_OIS;
		fs_sel |= (s->base_state.gyro_ois_fullscale << 5);
		result |= inv_icm20602_mems_write_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &fs_sel);

		data |= BIT_OIS_ENABLE;
	}
	else {
		data &= ~BIT_OIS_ENABLE;
	}

	result |= inv_icm20602_mems_write_reg(s, MPUREG_USER_CTRL_NEW, 1, &data);
#endif

	// gyro ODR range in low power mode is 3.91Hz ~ 333.33Hz
	if((smplrt_divider == 1) || ((smplrt_divider == 2) && (bit_mask & GYRO_AVAILABLE)))
		bit_mask |= HIGH_ODR_REQUESTED;

	switch(bit_mask & (ACCEL_AVAILABLE | GYRO_AVAILABLE | OIS_AVAILABLE | HIGH_ODR_REQUESTED)) {
	case 0 :
		// Procedure 0
		s->base_state.pwr_mgmt_1 |= BIT_SLEEP;
		result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);

		s->base_state.pwr_mgmt_2 |= BIT_PWR_ALL_OFF;
		result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
		
		s->base_state.wake_state &= ~CHIP_AWAKE;
		s->power_state = PowerStateSleepState;
		break;

	case ACCEL_AVAILABLE : // to accel Low Power
		// no need to go above 500Hz data rate so reduce power consumption
		// and put accel in Low Power mode in this case
		switch(s->power_state) 
		{
			case PowerStateSleepState:
				// Procedure 1
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				// change ACCEL_FS_SEL
				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20602_sleep(2);
				result |= apply_accel_fsr(s);

				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // LP mode not available for 20603
#else
				s->base_state.pwr_mgmt_1 |= BIT_CYCLE; // no gyro, accelero in LP
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.wake_state &= ~CHIP_LP_ENABLE; // LP mode not available for 20603
#else
				s->base_state.wake_state |= CHIP_LP_ENABLE;
#endif

				result |= apply_accel_bw(s);
				inv_icm20602_sleep(20); // accel startup time
				break;
			case PowerStateAccLNState:
				// Procedure 2
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // LP mode not available for 20603
#else
				s->base_state.pwr_mgmt_1 |= BIT_CYCLE; // no gyro, accelero in LP
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.wake_state &= ~CHIP_LP_ENABLE; // LP mode not available for 20603
#else
				s->base_state.wake_state |= CHIP_LP_ENABLE;
#endif
				result |= apply_accel_bw(s);
				break;
			case PowerStateGyrLPState:
			case PowerStateGyrLNState:
				//Procedure 4/5
				// change ACCEL_FS_SEL
				result |= apply_accel_fsr(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // LP mode not available for 20603
#else
				s->base_state.pwr_mgmt_1 |= BIT_CYCLE; // no gyro, accelero in LP
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.wake_state &= ~CHIP_LP_ENABLE; // LP mode not available for 20603
#else
				s->base_state.wake_state |= CHIP_LP_ENABLE;
#endif

#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 &= ~BIT_CLKSEL; // set clksel=0
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				inv_icm20602_sleep(1);
#endif
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 |= CLK_SEL; // set clksel=1
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#endif
				result |= apply_accel_bw(s);
				break;
			case PowerState6AxisLPState:
			case PowerState6AxisLNState:
				// Procedure 6/7
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // LP mode not available for 20603
#else
				s->base_state.pwr_mgmt_1 |= BIT_CYCLE; // no gyro, accelero in LP
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.wake_state &= ~CHIP_LP_ENABLE; // LP mode not available for 20603
#else
				s->base_state.wake_state |= CHIP_LP_ENABLE;
#endif

#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 &= ~BIT_CLKSEL; // set clksel=0
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				inv_icm20602_sleep(1);
#endif
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 |= CLK_SEL; // set clksel=1
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#endif
				result |= apply_accel_bw(s);
				break;
			default:
				break;
		}

#if (MEMS_CHIP == HW_ICM20603)
		s->power_state = PowerStateAccLNState;
#else
		s->power_state = PowerStateAccLPState;
#endif
		result |= apply_accel_bias(s);
		break;

	case ACCEL_AVAILABLE | HIGH_ODR_REQUESTED: // to accel accel Low Noise
		// accel needs 1kHz data rate since divider is 1 so need to have sample rate of 1kHz
		// which is only possible when accel Low Noise mode is enabled
		switch(s->power_state) 
		{
			case PowerStateSleepState:
				// Procedure 8
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20602_sleep(2);
				// change ACCEL_FS_SEL
				result |= apply_accel_fsr(s);
				result |= apply_accel_bw(s);
				inv_icm20602_sleep(20); // accel startup time
				break;
			case PowerStateAccLPState:
				// Procedure 9
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20602_sleep(2);
				result |= apply_accel_bw(s);
				break;
			case PowerStateGyrLNState:
			case PowerStateGyrLPState:
				// Procedure 11 & 12
				result |= apply_accel_bw(s);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 &= ~BIT_CLKSEL; // set clksel=0
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				inv_icm20602_sleep(1);
#endif
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 |= CLK_SEL; // set clksel=1
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#endif
				break;
			case PowerState6AxisLPState:
			case PowerState6AxisLNState:
				// Procedure 13
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 &= ~BIT_CLKSEL; // set clksel=0
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				inv_icm20602_sleep(1);
#endif
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
#if (MEMS_CHIP == HW_ICM20690)
				s->base_state.pwr_mgmt_1 |= CLK_SEL; // set clksel=1
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
#endif
				break;
			default:
				break;
		}

		s->power_state = PowerStateAccLNState;
		result |= apply_accel_bias(s);
		break;

	case GYRO_AVAILABLE : // to gyro Low Power
		// Turn on gyro only
		switch(s->power_state) 
		{
			case PowerStateSleepState:
				//Procedure 18
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // GYRO_STANDBY = 0 & CYCLE = 0 & SLEEP = 0
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20602_sleep(2);
				result |= apply_gyr_fsr_bw(s);
				inv_icm20602_sleep(50); // gyro startup time
				break;
			case PowerStateAccLPState:
				//Procedure 19
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20602_sleep(2);
				result |= apply_gyr_fsr_bw(s);
				break;
			case PowerStateAccLNState:
				//Procedure 20
				result |= apply_gyr_fsr_bw(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				break;
			case PowerStateGyrLNState:
				//Procedure 22

				// Gyro full scale should be reconfigured because OIS disable for 1st time
				result |= apply_gyr_fsr_bw(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				break;
			case PowerState6AxisLPState:
				//Procedure 23
				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				break;
			case PowerState6AxisLNState:
				//Procedure 24

				// Gyro full scale should be reconfigured because OIS disable for 1st time
				result |= apply_gyr_fsr_bw(s);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				break;
			default:
				break;
		}

#if (MEMS_CHIP == HW_ICM20603)
		s->power_state = PowerStateGyrLNState;
#else
		s->power_state = PowerStateGyrLPState;
#endif
		result |= apply_gyr_bias(s);
		break;

	case OIS_AVAILABLE: // to Gyro Low Noise
	case GYRO_AVAILABLE | OIS_AVAILABLE | HIGH_ODR_REQUESTED: // to Gyro Low Noise
	case GYRO_AVAILABLE | HIGH_ODR_REQUESTED: // to Gyro Low Noise
	case GYRO_AVAILABLE | OIS_AVAILABLE: // to Gyro Low Noise

		switch(s->power_state)
		{
			case PowerStateSleepState:
				//Procedure 25
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20602_sleep(2);

				result |= apply_gyr_fsr_bw(s);
				inv_icm20602_sleep(50); // gyro startup time
				break;
			case PowerStateAccLPState:
				//Procedure 26
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;
				inv_icm20602_sleep(2);
				result |= apply_gyr_fsr_bw(s);
				break;
			case PowerStateAccLNState:
				//Procedure 27
				result |= apply_gyr_fsr_bw(s);
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				break;
			case PowerStateGyrLPState:
				//Procedure 29

				// Gyro full scale should be reconfigured because OIS enabled for 1st time
				result |= apply_gyr_fsr_bw(s);

				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				break;
			case PowerState6AxisLPState:
				//Procedure 30

				// Gyro full scale should be reconfigured because OIS enabled for 1st time
				result |= apply_gyr_fsr_bw(s);

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
				break;
			case PowerState6AxisLNState:
				//Procedure 31

				// No need to reconfigure Gyro full scale because OIS was already enabled

				s->base_state.pwr_mgmt_2 |= BIT_PWR_ACCEL_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
				break;
			default:
				break;
		}

		s->power_state = PowerStateGyrLNState;
		result |= apply_gyr_bias(s);
		break;

	case ACCEL_AVAILABLE|GYRO_AVAILABLE : // to 6axis Low Power
		// Turn on gyro and accel so accel in low noise mode

		switch(s->power_state)
		{
			case PowerStateSleepState:
				//Procedure 32
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state |= CHIP_AWAKE;
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;

				inv_icm20602_sleep(2);

				result |= apply_accel_fsr(s);
				result |= apply_accel_bw(s);
				result |= apply_gyr_fsr_bw(s);
				inv_icm20602_sleep(50); // gyro startup time
			break;
			case PowerStateAccLPState:
				//Procedure 33
#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ALL_OFF; // Zero means all sensors are on
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

				s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE; // gyro on and accelero also, accelero in low-noise
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
				s->base_state.wake_state &= ~CHIP_LP_ENABLE;

				inv_icm20602_sleep(2);

				result |= apply_gyr_fsr_bw(s);
			break;
			case PowerStateAccLNState:
				//Procedure 34
				result |= apply_gyr_fsr_bw(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
			break;
			case PowerStateGyrLPState:
				//Procedure 36
				result |= apply_accel_fsr(s);
				result |= apply_accel_bw(s);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
			break;
			case PowerStateGyrLNState:
				//Procedure 37
				result |= apply_accel_fsr(s);
				result |= apply_accel_bw(s);

				// Gyro full scale should be reconfigured because OIS disable for 1st time
				result |= apply_gyr_fsr_bw(s);

				s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
				result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
			break;
			case PowerState6AxisLNState:
				//Procedure 38

				// Gyro full scale should be reconfigured because OIS disable for 1st time
				result |= apply_gyr_fsr_bw(s);

#if (MEMS_CHIP == HW_ICM20603)
				s->base_state.lp_config &= ~BIT_GYRO_CYCLE; // LP mode not available for 20603
#else
				s->base_state.lp_config |= BIT_GYRO_CYCLE;
#endif
				result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
			break;
			default :
			break;
		}

#if (MEMS_CHIP == HW_ICM20603)
		s->power_state = PowerState6AxisLNState;
#else
		s->power_state = PowerState6AxisLPState;
#endif
		result |= apply_accel_bias(s);
		result |= apply_gyr_bias(s);
		break;

	case ACCEL_AVAILABLE | OIS_AVAILABLE: // to 6axis Low Noise
	case ACCEL_AVAILABLE | OIS_AVAILABLE  | HIGH_ODR_REQUESTED: // to 6axis Low Noise
	case ACCEL_AVAILABLE | GYRO_AVAILABLE | HIGH_ODR_REQUESTED: // to 6axis Low Noise
	case ACCEL_AVAILABLE | GYRO_AVAILABLE | OIS_AVAILABLE: // to 6axis Low Noise
	case ACCEL_AVAILABLE | GYRO_AVAILABLE | OIS_AVAILABLE | HIGH_ODR_REQUESTED: // to 6axis Low Noise

		switch(s->power_state) {
		case PowerStateSleepState:
			//Procedure 39
			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

			s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
			s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
			s->base_state.wake_state |= CHIP_AWAKE;
			s->base_state.wake_state &= ~CHIP_LP_ENABLE;
			inv_icm20602_sleep(2);

			apply_accel_fsr(s);
			apply_accel_bw(s);
			apply_gyr_fsr_bw(s);
			inv_icm20602_sleep(50); // gyro startup time
			break;
		case PowerStateAccLPState:
			//Procedure 40
			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

			s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
			s->base_state.wake_state &= ~CHIP_LP_ENABLE;
			inv_icm20602_sleep(2);

			result |= apply_gyr_fsr_bw(s);
			break;
		case PowerStateAccLNState:
			//Procedure 41
			result |= apply_gyr_fsr_bw(s);

			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);

			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_GYRO_STBY;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
			break;
		case PowerStateGyrLPState:
			//Procedure 43
			result |= apply_accel_fsr(s);
			result |= apply_accel_bw(s);

			// Gyro full scale should be reconfigured because OIS enabled for 1st time
			result |= apply_gyr_fsr_bw(s);

			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
			break;
		case PowerStateGyrLNState:
			//Procedure 44
			result |= apply_accel_fsr(s);
			result |= apply_accel_bw(s);

			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);

			// No need to reconfigure Gyro full scale because OIS was already enabled
			break;
		case PowerState6AxisLPState:
			//Procedure 45

			// Gyro full scale should be reconfigured because OIS enabled for 1st time
			result |= apply_gyr_fsr_bw(s);

			s->base_state.lp_config &= ~BIT_GYRO_CYCLE;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_LP_CONFIG, 1, &s->base_state.lp_config);
			break;
		default:
			break;
		}

		s->power_state = PowerState6AxisLNState;
		result |= apply_accel_bias(s);
		result |= apply_gyr_bias(s);
		break;

	case HIGH_ODR_REQUESTED: // no sensor ON
	default:
		break;
	}

#if (MEMS_CHIP == HW_ICM20690)
	/* overwrite pwr_mgmt_1 according to AUX I2C usage */
	if (bit_mask & COMPASS_AVAILABLE) {
		s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
		s->base_state.pwr_mgmt_1 &= ~BIT_CYCLE;
		result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
		s->base_state.wake_state |= CHIP_AWAKE;
		s->base_state.wake_state &= ~CHIP_LP_ENABLE;
		result |= inv_icm20602_set_accel_bandwidth(s, MPU_ABW_218);
		apply_accel_bw(s);
		inv_icm20602_sleep(20); // accel startup time
		if ((s->base_state.pwr_mgmt_2 & BIT_PWR_ALL_OFF) == BIT_PWR_ALL_OFF) {
			/* need either accel or gyro engine on for AUX I2C */
			s->base_state.pwr_mgmt_2 &= ~BIT_PWR_ACCEL_STBY;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
		}
		result |= inv_icm20602_resume_akm(s);
	} else {
		if (s->power_state == PowerStateAccLPState) {
			/* return to accel lp mode */
			s->base_state.pwr_mgmt_1 |= BIT_CYCLE;
			result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);
			s->base_state.wake_state |= CHIP_LP_ENABLE;
			result |= inv_icm20602_set_accel_bandwidth(s, MPU_ABW_420);
			apply_accel_bw(s);
		}
		result |= inv_icm20602_suspend_akm(s);
	}
#endif

	/* workaround to avoid impact on gyro offsets when enable/disable temp (#14339) */
	s->base_state.pwr_mgmt_1 &= ~BIT_TEMP_DISABLE; // Turn ON temp 
	result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &s->base_state.pwr_mgmt_1);

	return result;
}

/******************/
typedef	struct {
	enum ANDROID_SENSORS           AndroidSensor;
	enum inv_icm20602_sensor InvSensor;
} MinDelayGenElementT;

/******************/
//	accel
static const MinDelayGenElementT MinDelayGenAccelList[]	= {
	{ ANDROID_SENSOR_ACCELEROMETER, INV_ICM20602_SENSOR_ACCEL }
};

//	gyro
static const MinDelayGenElementT MinDelayGenGyroList[] = {
	{ ANDROID_SENSOR_GYROSCOPE, INV_ICM20602_SENSOR_GYRO }
};

//	compass
static const MinDelayGenElementT MinDelayGenCompassList[] = {
	{ ANDROID_SENSOR_MAGNETOMETER, INV_ICM20602_SENSOR_COMPASS }
};


typedef	unsigned	short	MinDelayT;
typedef	unsigned	short	u16;
typedef	unsigned	long	u32;

/******************/
#define	MinDelayGen(s, list)	MinDelayGenActual((s), list, sizeof(list) / sizeof (MinDelayGenElementT))

static MinDelayT MinDelayGenActual(struct inv_icm20602 * s, const MinDelayGenElementT * Element, u32 ElementQuan)
{
	MinDelayT MinDelay = (MinDelayT)-1;

	while(ElementQuan--) {
		if(is_android_sensor_enabled(s, Element->AndroidSensor)) {
			MinDelayT OdrDelay = s->requested_odr[Element->InvSensor];
			if(MinDelay > OdrDelay) {
				MinDelay = OdrDelay;
			}
		}
		Element++;
	} //	end while elements to process

	return	MinDelay;
}

/******************/
static u16 SampleRateDividerGet(struct inv_icm20602 * s, MinDelayT MinDelay)
{
	//u16	Delay = INV_MIN(ODR_MIN_DELAY, MinDelay);
	return MinDelay * inv_icm20602_get_chip_base_sample_rate(s)/1000L; // a divider from BASE_SAMPLE_RATE Hz.
}

/******************/
static int set_hw_smplrt_dmp_odrs(struct inv_icm20602 * s, uint16_t * resulting_divider)
{
	int result = 0;
	uint8_t data = 0;
	uint16_t hw_smplrt_divider = 0;

	// Set UI ODR (always <=1kHz)
	// Set accel_fchoice_b to 0
	if(s->android_sensor_mask & INV_NEEDS_ACCEL_MASK)
	{
		result |= inv_icm20602_mems_read_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &data);
		data &= ~BITS_ACCEL_FCHOICE_B; // Clear ACCEL_FCHOICE_B
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_CONFIG_2, 1, &data);
	}

	// Set fchoice_b to 0, dlpf_cfg to 1
	if(s->android_sensor_mask & INV_NEEDS_GYRO_MASK)
	{
		result |= inv_icm20602_mems_read_reg(s, MPUREG_GYRO_CONFIG, 1, &data);
		data &= ~BITS_FCHOICE_B; // Clear FCHOICE_B
		result |= inv_icm20602_mems_write_reg(s, MPUREG_GYRO_CONFIG, 1, &data);

		result |= inv_icm20602_mems_read_reg(s, MPUREG_CONFIG, 1, &data);
		data &= ~0x7; // Clear bits[2:0]
		data |= 0x1;
		result |= inv_icm20602_mems_write_reg(s, MPUREG_CONFIG, 1, &data);
	}

	// Set sample rate divider
	if((s->android_sensor_mask & INV_NEEDS_ACCEL_MASK) ||
			(s->android_sensor_mask & INV_NEEDS_GYRO_MASK) ||
			(s->android_sensor_mask & INV_NEEDS_COMPASS_MASK))
	{
		uint16_t minDly, minDly_accel, minDly_gyro, minDly_compass;
		// get min delays of all enabled sensors for each sensor engine group

		// Engine ACCEL Based
		minDly_accel = MinDelayGen(s, MinDelayGenAccelList);

		// Engine Gyro Based
		minDly_gyro = MinDelayGen(s, MinDelayGenGyroList);

		// Engine I2C MST Based
		minDly_compass = MinDelayGen(s, MinDelayGenCompassList);
		// Limit compass max access rate
		minDly_compass = INV_MAX(minDly_compass, COMPASS_MIN_DLY); 

		// get min delay of all enabled sensors of all sensor engine groups
		minDly = INV_MIN(minDly_gyro, minDly_accel);
		if(minDly>minDly_compass) // if compass is faster
			s->base_state.sample_repeat_cnt = minDly/minDly_compass;
		else
			s->base_state.sample_repeat_cnt = 1;

		minDly = INV_MIN(minDly, minDly_compass);

		// set odrs for each enabled sensors
		if (minDly != 0xFFFF)
		{
			hw_smplrt_divider = SampleRateDividerGet(s, minDly);
			result |= inv_icm20602_set_divider(s, (uint8_t)(hw_smplrt_divider - 1));
			if(s->android_sensor_mask & INV_NEEDS_COMPASS_MASK)
				result |= inv_icm20602_set_i2c_mst_dly(s, minDly, minDly_compass);
		}
	}

	// Set OIS ODR
#if (MEMS_CHIP == HW_ICM20690)
	// TODO: Hardcode OIS Gyro ODR to 8kHz and OIS accel ODR to 4kHz for now.
	if (s->android_sensor_mask & INV_NEEDS_OIS_MASK) {
		// Set fchoice_ois_b to 01
		result |= inv_icm20602_mems_read_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &data);
		data &= ~BITS_FCHOICE_OIS_B;
		data |= (1 << BIT_POS_FCHOICE_OIS_B);
		result |= inv_icm20602_mems_write_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &data);

		// Set accel_fchoice_ois_b to 11
		result |= inv_icm20602_mems_read_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);
		data &= ~BITS_ACCEL_FCHOICE_OIS_B;
		data |= (3 << BIT_POS_ACCEL_FCHOICE_OIS_B);
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);
	}
#endif

	*resulting_divider = hw_smplrt_divider;
	return result;
}

int inv_icm20602_set_sensor_period(struct inv_icm20602 * s, enum inv_icm20602_sensor sensor, uint16_t delayInMs)
{
	int result = 0;
	uint16_t resulting_divider = 0;
	uint16_t data_rdy_status = 0;

	const uint8_t androidSensor = sensor_type_2_android_sensor(sensor);

	if(delayInMs < ODR_MIN_DELAY) delayInMs = ODR_MIN_DELAY;
	if(delayInMs > ODR_MAX_DELAY) delayInMs = ODR_MAX_DELAY;

	switch (androidSensor) {
	case ANDROID_SENSOR_ACCELEROMETER:
		s->requested_odr[INV_ICM20602_SENSOR_ACCEL] = delayInMs;
		break;
	case ANDROID_SENSOR_GYROSCOPE:
		s->requested_odr[INV_SENSOR_GYRO] = delayInMs;
		break;
	case ANDROID_SENSOR_MAGNETOMETER:
		s->requested_odr[INV_SENSOR_COMPASS] = delayInMs;
		break;
		// not support yet
	case ANDROID_SENSOR_TEMPERATURE:
		break;
	default:
		break;
	}
	result |= inv_icm20602_disable_fifo(s);
	result |= set_hw_smplrt_dmp_odrs(s, &resulting_divider);

	if (s->android_sensor_mask & INV_NEEDS_GYRO_MASK)
		data_rdy_status |= GYRO_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_ACCEL_MASK)
		data_rdy_status |= ACCEL_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_COMPASS_MASK)
		data_rdy_status |= COMPASS_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_TEMP_MASK)
		data_rdy_status |= TEMP_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_OIS_MASK)
		data_rdy_status |= (OIS_AVAILABLE | GYRO_AVAILABLE);

	// we reconfigured ODR, so there is a chance that we change from LP to/from LN noise, just check it
	result |= inv_icm20602_enable_mems(s, (int)data_rdy_status, resulting_divider);

	// need to reset and restart FIFO with new configurations
	result |= inv_icm20602_enable_fifo(s, (int)data_rdy_status);

	return result;
}

int inv_icm20602_is_sensor_enabled(struct inv_icm20602 * s, enum inv_icm20602_sensor sensor)
{
	const uint8_t androidSensor = sensor_type_2_android_sensor(sensor);

	if(androidSensor < ANDROID_SENSOR_NUM_MAX) {
		return (is_android_sensor_enabled(s, androidSensor) != 0);
	}

	return ANDROID_SENSOR_NUM_MAX;
}

static void regenerate_sensor_control(struct inv_icm20602 * s,
		const short * sen_num_2_ctrl, uint16_t *sensor_control)
{
	short delta;
	int cntr = 0;
	uint32_t tmp_androidSensorsOn_mask;

	*sensor_control = 0;
	tmp_androidSensorsOn_mask = s->android_sensor_mask;
	while (tmp_androidSensorsOn_mask) {
		if (tmp_androidSensorsOn_mask & 1) {
			delta = sen_num_2_ctrl[cntr];
			if (delta != -1) *sensor_control |= delta;
		}
		tmp_androidSensorsOn_mask >>= 1;
		cntr++;
	}
}

/** Computes the sensor control register that needs to be sent to the DMP
 *  @param[in] androidSensor A sensor number, the numbers correspond to sensors.h definition in Android
 *  @param[in] enable non-zero to turn sensor on, 0 to turn sensor off
 *  @param[in] sen_num_2_ctrl Table matching android sensor number to bits in DMP control register
 *  @param[in,out] sensor_control Sensor control register to write to DMP to enable/disable sensors
 */
static void convert_android_sensor_to_control(struct inv_icm20602 * s,
	uint8_t androidSensor, uint8_t enable, const short *sen_num_2_ctrl, uint16_t *sensor_control)
{
	short delta = 0;

	if (androidSensor >= ANDROID_SENSOR_NUM_MAX)
		return; // Sensor not supported

	delta = sen_num_2_ctrl[androidSensor];
	if (delta == -1)
		return; // This sensor not supported

	if (enable) {
		s->android_sensor_mask |= 1L << (androidSensor & 0x1F); // Set bit
		*sensor_control |= delta;
	} else {
		s->android_sensor_mask &= ~(1L << (androidSensor & 0x1F)); // Clear bit
		// control has to be regenerated when removing sensors because of overlap
		regenerate_sensor_control(s, sen_num_2_ctrl, sensor_control);
	}
}

int inv_icm20602_enable_sensor(struct inv_icm20602 * s, enum inv_icm20602_sensor sensor, uint8_t enable)
{
	int result = 0;
	uint16_t data_rdy_status = 0;
	uint16_t resulting_divider = 0;
	uint16_t sensor_control = 0;

	static const int16_t android_sensor_2_control_bits[ANDROID_SENSOR_NUM_MAX]= {
		// Unsupported Sensors are -1
		0x8008, // Accelerometer
		0x0048, // Gyroscope
		0x0048, // OIS
		0x8008, // Temperature
		0x0048, // FSYNC
	};

	const uint8_t androidSensor = sensor_type_2_android_sensor(sensor);

	convert_android_sensor_to_control(s, androidSensor, enable,
			android_sensor_2_control_bits, &sensor_control);

	if (s->android_sensor_mask & INV_NEEDS_GYRO_MASK)
		data_rdy_status |= GYRO_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_ACCEL_MASK)
		data_rdy_status |= ACCEL_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_COMPASS_MASK)
		data_rdy_status |= COMPASS_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_TEMP_MASK)
		data_rdy_status |= TEMP_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_OIS_MASK)
		data_rdy_status |= OIS_AVAILABLE;
	if (s->android_sensor_mask & INV_NEEDS_FSYNC_MASK) {
		inv_icm20602_set_fsync_bit_location(s, enable);
	}

	// Configure interrupt
	result |= enable_interrupt(s, (int)data_rdy_status);

	// turn on gyro cal only if gyro is available
	result |= inv_icm20602_disable_fifo(s);
	result |= set_hw_smplrt_dmp_odrs(s, &resulting_divider);
	result |= inv_icm20602_enable_mems(s, (int)data_rdy_status, resulting_divider);
	// need to reset and restart FIFO with new configurations
	result |= inv_icm20602_enable_fifo(s, (int)data_rdy_status);
	// drop the first data in the FIFO
	s->dropped_data[sensor] = 1;
	if(s->dropped_data[INV_ICM20602_SENSOR_ACCEL] && s->dropped_data[INV_ICM20602_SENSOR_GYRO]) {
		s->dropped_data[INV_ICM20602_SENSOR_ACCEL] = 2;
		s->dropped_data[INV_ICM20602_SENSOR_GYRO] = 2;
	}

	return result;
}

int inv_icm20602_configure_accel_wom(struct inv_icm20602 * s, uint8_t wom_threshold)
{
	int result = 0;
	uint8_t data;

	if(wom_threshold) {
		// Set threshold
		// wom_threshold = 0x0F;
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_WOM_THR,   1, &wom_threshold);
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_WOM_X_THR, 1, &wom_threshold);
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_WOM_Y_THR, 1, &wom_threshold);
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_WOM_Z_THR, 1, &wom_threshold);

		// Enable wake on motion interrupt on all 3 axes
		result |= inv_icm20602_mems_read_reg(s, MPUREG_INT_ENABLE, 1, &data);
		data |= 0xE0;
		data &= ~BIT_DATA_RDY_INT_EN; // clear data ready interrupt
		result |= inv_icm20602_mems_write_reg(s, MPUREG_INT_ENABLE, 1, &data);

		// Enable WOM logic and set WOM interrupt mode
		// bit 7: Enable the WOM logic
		// bit 6: 1: compare the current sample with the previous sample.
		//        0: initial sample is stored, all future samples are compared to the initial sample.
		// bit 0: 0 – Set WoM interrupt on OR of x,y or z axis interrupt
		//        1 – Set WoM interrupt on AND of x, y and z axes interrupts
		result |= inv_icm20602_mems_read_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);
		data |= 0xC0; // Compare current sample with the previous sample.
		              // Set WoM interrupt on OR of xyz axes interrupt.
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);

		// Wake up chip and set to LP mode
		result |= inv_icm20602_mems_read_reg(s, MPUREG_PWR_MGMT_1, 1, &data);
		data &= ~BIT_SLEEP; // Clear sleep bit
		data |= BIT_CYCLE; // Enable accel cycling mode.
		result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &data);

		// Enable accel
		result |= inv_icm20602_mems_read_reg(s, MPUREG_PWR_MGMT_2, 1, &data);
		data &= ~BIT_PWR_ACCEL_STBY; // Enable accel
		data |= BIT_PWR_GYRO_STBY; // Disable gyro.
		result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &data);

		s->wom_enabled = 1;
	}
	else {
		uint16_t resulting_divider = 0;
		uint16_t data_rdy_status = 0;

		// Disable wake on motion interrupt on all 3 axes
		result |= inv_icm20602_mems_read_reg(s, MPUREG_INT_ENABLE, 1, &data);
		data &= ~0xE0;
		data |= BIT_DATA_RDY_INT_EN; // Set data ready interrupt
		result |= inv_icm20602_mems_write_reg(s, MPUREG_INT_ENABLE, 1, &data);

		// Disable WOM logic and set WOM interrupt mode
		result |= inv_icm20602_mems_read_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);
		data &= ~0x80; // Disable WoM logic
		result |= inv_icm20602_mems_write_reg(s, MPUREG_ACCEL_INTEL_CTRL, 1, &data);

		// Disable accel
		result |= inv_icm20602_mems_read_reg(s, MPUREG_PWR_MGMT_2, 1, &data);
		data |= BIT_PWR_ACCEL_STBY; // Disable accel
		result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_2, 1, &data);

		// Put chip to sleep
		result |= inv_icm20602_mems_read_reg(s, MPUREG_PWR_MGMT_1, 1, &data);
		data |= BIT_SLEEP; // Set sleep bit
		result |= inv_icm20602_mems_write_reg(s, MPUREG_PWR_MGMT_1, 1, &data);
		s->power_state = PowerStateSleepState;

		// Re-enable sensors with data ready interrupt configuration
		if (s->android_sensor_mask & INV_NEEDS_GYRO_MASK) 
			data_rdy_status |= GYRO_AVAILABLE;
		if (s->android_sensor_mask & INV_NEEDS_ACCEL_MASK)
			data_rdy_status |= ACCEL_AVAILABLE;
		if (s->android_sensor_mask & INV_NEEDS_COMPASS_MASK)
			data_rdy_status |= COMPASS_AVAILABLE;
		if (s->android_sensor_mask & INV_NEEDS_TEMP_MASK)
			data_rdy_status |= TEMP_AVAILABLE;

		result |= inv_icm20602_disable_fifo(s);
		result |= set_hw_smplrt_dmp_odrs(s, &resulting_divider);
		result |= inv_icm20602_enable_mems(s, (int)data_rdy_status, resulting_divider);
		// need to reset and restart FIFO with data ready interrupt configuration
		result |= inv_icm20602_enable_fifo(s, (int)data_rdy_status);

		s->wom_enabled = 0;
	}

	return result;
}


int inv_icm20602_disable_fifo(struct inv_icm20602 * s)
{
#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20603)||(MEMS_CHIP == HW_ICM20789)||(MEMS_CHIP == HW_ICM20609))
	int result = 0;
	uint8_t reg;

	/* disable FIFO */
	result |= inv_icm20602_mems_read_reg(s, MPUREG_USER_CTRL, 1, &reg);
	reg &= ~BIT_FIFO_EN;
	result |= inv_icm20602_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	/* reset FIFO_EN as well */
	reg = 0;
	result |= inv_icm20602_mems_write_reg(s, MPUREG_FIFO_EN, 1, &reg);

	return result;
#else
	(void)s;

	return 0;
#endif
}

int inv_icm20602_enable_fifo(struct inv_icm20602 * s, int bit_mask)
{
#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20603)||(MEMS_CHIP == HW_ICM20789)||(MEMS_CHIP == HW_ICM20609))
	int result = 0;
	uint8_t reg;
	uint8_t user_ctrl;

	/* no need to enable FIFO as no data to push */
	if (bit_mask == 0) {
		inv_icm20602_disable_fifo(s);
		return 0;
	}

	/* stop and reset FIFO */
	result |= inv_icm20602_mems_read_reg(s, MPUREG_USER_CTRL, 1, &user_ctrl);
	reg = user_ctrl | BIT_FIFO_RST;
	reg &= ~BIT_FIFO_EN;
	result |= inv_icm20602_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	/* push required sensor data to FIFO */
	reg = 0;
#if ((MEMS_CHIP == HW_ICM20789) || (MEMS_CHIP == HW_ICM20609))
	if (bit_mask & GYRO_AVAILABLE)
		reg |= BIT_XGYRO_OUT|BIT_YGYRO_OUT|BIT_ZGYRO_OUT;
#else
	if (bit_mask & GYRO_AVAILABLE)
		reg |= BIT_GYRO_OUT;
#endif
	if (bit_mask & ACCEL_AVAILABLE)
		reg |= BIT_ACCEL_OUT;
#if ((MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20789)||(MEMS_CHIP == HW_ICM20609))
	if (bit_mask & TEMP_AVAILABLE)
		reg |= BIT_TEMP_OUT;
#endif
#if (MEMS_CHIP == HW_ICM20690)
	if (bit_mask & COMPASS_AVAILABLE) {
		switch (s->secondary_state.compass_aux_ch) {
			case 0:
				reg |= BIT_SLV0_OUT;
				break;
			case 1:
				reg |= BIT_SLV1_OUT;
				break;
			case 2:
				reg |= BIT_SLV2_OUT;
				break;
			default:
				reg |= BIT_SLV0_OUT;
				break;
		}
	}
#endif

	result |= inv_icm20602_mems_write_reg(s, MPUREG_FIFO_EN, 1, &reg);

	/* restart FIFO */
	reg = user_ctrl | BIT_FIFO_EN;
	result |= inv_icm20602_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	return result;
#else

	(void)s, (void)bit_mask;

	return 0;
#endif
}

int inv_icm20602_reset_fifo(struct inv_icm20602 * s)
{
#if ((MEMS_CHIP == HW_ICM20602)||(MEMS_CHIP == HW_ICM20690)||(MEMS_CHIP == HW_ICM20603)||(MEMS_CHIP == HW_ICM20789)||(MEMS_CHIP == HW_ICM20609))
	int result = 0;
	uint8_t reg;

	result |= inv_icm20602_mems_read_reg(s, MPUREG_USER_CTRL, 1, &reg);
	reg |= BIT_FIFO_RST;
	result |= inv_icm20602_mems_write_reg(s, MPUREG_USER_CTRL, 1, &reg);

	return result;
#else
	(void)s;

	return 0;
#endif
}

int inv_icm20602_set_slave_compass_id(struct inv_icm20602 * s)
{
	int result = 0;

#if (MEMS_CHIP == HW_ICM20690)
	/* initialize aux i2c */
	inv_icm20602_init_secondary(s);
	/* initialize akm compass */
	result |= inv_icm20602_setup_compass_akm(s);
#else
	(void)s;
#endif

	return result;
}

// only read gyro data for now
int inv_icm20602_poll_ois_gyro_data(struct inv_icm20602 * s, int16_t gyro_data[3])
{
#if (MEMS_CHIP == HW_ICM20690)
	uint8_t data[6];
	int rc = 0;

	if ((s->android_sensor_mask & INV_NEEDS_OIS_MASK && s->use_serif_ois))
	{
		if(inv_icm20602_read_reg_ois(s, 8, data, 6)==0) {
			/* raw sensor data output is in big endian */
			gyro_data[0] = (data[0] << 8) | data[1];
			gyro_data[1] = (data[2] << 8) | data[3];
			gyro_data[2] = (data[4] << 8) | data[5];

			rc |= (1 << INV_ICM20602_SENSOR_OIS);
		}
	}
	return rc;
#else
	(void)s, (void)gyro_data;
	return 0;
#endif
}

int inv_icm20602_poll_delay_count(struct inv_icm20602 * s, int16_t * delay_count)
{
#if (MEMS_CHIP == HW_ICM20690)
	uint8_t data[2];
	int rc = 0;
	if (inv_icm20602_read_reg(s, MPUREG_TEMP_XOUT_L, data, 1) == 0) {
		if (data[0] & 0x1 /* keep the eis flag (bit 0) */) {

			rc |= inv_icm20602_read_reg(s, MPUREG_ODR_DLY_CNT_HI, data, 2);
			*delay_count = (data[0] << 8) | data[1];

			rc |= (1 << INV_ICM20602_SENSOR_FSYNC_EVENT);
		}
	}
	
	return rc;
#else
	(void)s, 
	*delay_count = 0;

	return INV_ERROR_NIMPL;
#endif
}

// get if all sensors are off
int inv_icm20602_all_sensors_off(struct inv_icm20602 * s)
{
	if( (s->android_sensor_mask == 0) && (s->wom_enabled == 0) )
		return 1;
	return 0;
}
