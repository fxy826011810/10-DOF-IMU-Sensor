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

#include "Icm20602AuxCompassAkm.h"
#include "Icm20602AuxTransport.h"
#include "Icm20602Transport.h"
#include "Icm20602Serif.h"
#include "Icm20602Defs.h"
#include "Icm20602.h"

#include "Invn/EmbUtils/Message.h"

/* AKM definitions */
#define REG_AKM_ID               0x00
#define REG_AKM_INFO             0x01
#define REG_AKM_STATUS           0x02
#define REG_AKM_MEASURE_DATA     0x03
#define REG_AKM_MODE             0x0A
#define REG_AKM_ST_CTRL          0x0C
#define REG_AKM_SENSITIVITY      0x10
#define REG_AKM8963_CNTL1        0x0A

/* AK09911 register definition */
#define REG_AK09911_DMP_READ    0x3
#define REG_AK09911_STATUS1     0x10
#define REG_AK09911_CNTL2       0x31
#define REG_AK09911_SENSITIVITY 0x60
#define REG_AK09911_MEASURE_DATA     0x11

/* AK09912 register definition */
#define REG_AK09912_DMP_READ    0x3
#define REG_AK09912_STATUS1     0x10
#define REG_AK09912_CNTL1       0x30
#define REG_AK09912_CNTL2       0x31
#define REG_AK09912_SENSITIVITY 0x60
#define REG_AK09912_MEASURE_DATA     0x11

#define DATA_AKM_ID              0x48
#define DATA_AKM_MODE_PD	 0x00
#define DATA_AKM_MODE_SM	 0x01
#define DATA_AKM_MODE_ST	 0x08
#define DATA_AK09911_MODE_ST	 0x10
#define DATA_AK09912_MODE_ST	 0x10
#define DATA_AKM_MODE_FR	 0x0F
#define DATA_AK09911_MODE_FR     0x1F
#define DATA_AK09912_MODE_FR     0x1F
#define DATA_AKM_SELF_TEST       0x40
#define DATA_AKM_DRDY            0x01
#define DATA_AKM8963_BIT         0x10
#define DATA_AKM_STAT_MASK       0x0C

#define DATA_AKM_WHOAMI_9911     0x5
#define DATA_AKM_WHOAMI_9912     0x4

/* 0.3 uT * (1 << 30) */
#define DATA_AKM8975_SCALE       322122547
/* 0.6 uT * (1 << 30) */
#define DATA_AKM8972_SCALE       644245094
/* 0.6 uT * (1 << 30) */
#define DATA_AKM8963_SCALE0      644245094
/* 0.15 uT * (1 << 30) */
#define DATA_AKM8963_SCALE1      161061273
/* 0.6 uT * (1 << 30) */
#define DATA_AK09911_SCALE       644245094
/* 0.15 uT * (1 << 30) */
#define DATA_AK09912_SCALE       161061273

#define DATA_AKM8963_SCALE_SHIFT      4
#define DATA_AKM_MIN_READ_TIME            (9 * NSEC_PER_MSEC)

/* AK09912C NSF */
/* 0:disable, 1:Low, 2:Middle, 3:High */
#define DATA_AK9912_NSF  1
#define DATA_AK9912_NSF_SHIFT 5

#define DEF_ST_COMPASS_WAIT_MIN     (10 * 1000)
#define DEF_ST_COMPASS_WAIT_MAX     (15 * 1000)
#define DEF_ST_COMPASS_TRY_TIMES    10
#define DEF_ST_COMPASS_8963_SHIFT   2
#define X                           0
#define Y                           1
#define Z                           2

/* milliseconds between each access */
#define AKM_RATE_SCALE       10

#define DATA_AKM_99_BYTES_DMP   10
#define DATA_AKM_89_BYTES_DMP   9

#if (MEMS_CHIP == HW_ICM20690)

static const short AKM8975_ST_Lower[3] = {-100, -100, -1000};
static const short AKM8975_ST_Upper[3] = {100, 100, -300};

static const short AKM8972_ST_Lower[3] = {-50, -50, -500};
static const short AKM8972_ST_Upper[3] = {50, 50, -100};

static const short AKM8963_ST_Lower[3] = {-200, -200, -3200};
static const short AKM8963_ST_Upper[3] = {200, 200, -800};

static const short AK09911_ST_Lower[3] = {-30, -30, -400};
static const short AK09911_ST_Upper[3] = {30, 30, -50};
static const short AK09912_ST_Lower[3] = {-200, -200, -1600};
static const short AK09912_ST_Upper[3] = {200, 200, -400};

#endif

void inv_icm20602_register_aux_compass(struct inv_icm20602 * s,
		enum inv_icm20602_compass_id compass_id, uint8_t compass_i2c_addr)
{
	/* only for 20690 for now */

#if (MEMS_CHIP == HW_ICM20690)
	switch(compass_id) {
	case INV_ICM20602_COMPASS_ID_AK09911:
		s->secondary_state.compass_id       = HW_AK09911;
		s->secondary_state.compass_i2c_addr = compass_i2c_addr;
		break;
	case INV_ICM20602_COMPASS_ID_AK09912:
		s->secondary_state.compass_id       = HW_AK09912;
		s->secondary_state.compass_i2c_addr = compass_i2c_addr;
		break;
	default:
		s->secondary_state.compass_id       = 0;
		s->secondary_state.compass_i2c_addr = 0;
	}
#else
	s->secondary_state.compass_id       = 0;
	s->secondary_state.compass_i2c_addr = 0;

	(void)s, (void)compass_id, (void)compass_i2c_addr;
#endif
}

int inv_icm20602_is_compass_registered(struct inv_icm20602 * s)
{
	return (s->secondary_state.compass_id != 0);
}

#if (MEMS_CHIP == HW_ICM20690)
/*
 *  inv_setup_compass_akm() - Configure akm series compass.
 */
int inv_icm20602_setup_compass_akm(struct inv_icm20602 * s)
{
	int result;
	unsigned char data[4];
	uint8_t sens, cmd, exp_whoami;

	/* Read WHOAMI through I2C SLV for compass */
	result = inv_icm20602_execute_read_secondary(s, COMPASS_I2C_SLV_READ,
			s->secondary_state.compass_i2c_addr, REG_AKM_ID, 2, data);
	if (result)
		return result;

	switch(s->secondary_state.compass_id){
	case HW_AK09911:
		exp_whoami = DATA_AKM_WHOAMI_9911;
		break;
	case HW_AK09912:
		exp_whoami = DATA_AKM_WHOAMI_9912;
		break;
	default:
		return -1;
	}

	if ((data[0] != DATA_AKM_ID) || (data[1] != exp_whoami)) {
		/* clear cached ID to not cause further error */
		INV_MSG(INV_MSG_LEVEL_WARNING, "Auxiliary AKM compass not found.");
		s->secondary_state.compass_id = 0;
		return -1;
	}

	/* setup upper and lower limit of self-test */
	if (HW_AK8975 == s->secondary_state.compass_id) {
		s->secondary_state.st_upper = (short*)AKM8975_ST_Upper;
		s->secondary_state.st_lower = (short*)AKM8975_ST_Lower;
	} else if (HW_AK8972 == s->secondary_state.compass_id) {
		s->secondary_state.st_upper = (short*)AKM8972_ST_Upper;
		s->secondary_state.st_lower = (short*)AKM8972_ST_Lower;
	} else if (HW_AK8963 == s->secondary_state.compass_id) {
		s->secondary_state.st_upper = (short*)AKM8963_ST_Upper;
		s->secondary_state.st_lower = (short*)AKM8963_ST_Lower;
	}  else if (HW_AK09911 == s->secondary_state.compass_id) {
		s->secondary_state.st_upper = (short*)AK09911_ST_Upper;
		s->secondary_state.st_lower = (short*)AK09911_ST_Lower;
	}  else if (HW_AK09912 == s->secondary_state.compass_id) {
		s->secondary_state.st_upper = (short*)AK09912_ST_Upper;
		s->secondary_state.st_lower = (short*)AK09912_ST_Lower;
	} else {
		return -1;
	}

	/* Read conf and configure compass through I2C SLV for compass and subsequent channel */

	/* set AKM to Fuse ROM access mode */
	if (HW_AK09911 == s->secondary_state.compass_id) {
		s->secondary_state.mode_reg_addr = REG_AK09911_CNTL2;
		sens = REG_AK09911_SENSITIVITY;
		cmd = DATA_AK09911_MODE_FR;
	} else if (HW_AK09912 == s->secondary_state.compass_id) {
		s->secondary_state.mode_reg_addr = REG_AK09912_CNTL2;
		sens = REG_AK09912_SENSITIVITY;
		cmd = DATA_AK09912_MODE_FR;
	} else {
		s->secondary_state.mode_reg_addr = REG_AKM_MODE;
		sens = REG_AKM_SENSITIVITY;
		cmd = DATA_AKM_MODE_FR;
	}

	result = inv_icm20602_read_secondary(s, COMPASS_I2C_SLV_READ,
			s->secondary_state.compass_i2c_addr, sens, THREE_AXES);
	if (result)
		return result;

	result = inv_icm20602_execute_write_secondary(s, COMPASS_I2C_SLV_WRITE,
			s->secondary_state.compass_i2c_addr, s->secondary_state.mode_reg_addr, cmd);
	if (result)
		return result;
	
	result = inv_icm20602_mems_read_reg(s, MPUREG_EXT_SLV_SENS_DATA_00,
			THREE_AXES, s->secondary_state.compass_sens);
	if (result)
		return result;

	if (HW_AK09912 == s->secondary_state.compass_id) {
		result = inv_icm20602_execute_write_secondary(s, COMPASS_I2C_SLV_WRITE,
				s->secondary_state.compass_i2c_addr, REG_AK09912_CNTL1, DATA_AK9912_NSF << DATA_AK9912_NSF_SHIFT);
		if (result)
			return result;
	}
	
	/* Set compass in power down through I2C SLV for compass */
	result = inv_icm20602_execute_write_secondary(s, COMPASS_I2C_SLV_WRITE,
			s->secondary_state.compass_i2c_addr, s->secondary_state.mode_reg_addr, DATA_AKM_MODE_PD);
	if (result)
		return result;

	s->secondary_state.compass_aux_ch = COMPASS_I2C_SLV_READ;
	s->secondary_state.secondary_resume_state = 1;

	return inv_icm20602_suspend_akm(s);
}

int inv_icm20602_check_akm_self_test(struct inv_icm20602 * s)
{
	int result;
	unsigned char data[6], mode, addr, d;
	unsigned char counter, cntl;
	short x, y, z;
	unsigned char *sens;
	int shift;
	unsigned char slv_ctrl[2];

	addr = s->secondary_state.compass_i2c_addr;
	sens = s->secondary_state.compass_sens;

	/* back up registers */
	/* SLV0_CTRL */
	result = inv_icm20602_mems_read_reg(s, MPUREG_I2C_SLV0_CTRL, 1, &slv_ctrl[0]);
	if (result)
		return result;
	d = 0;
	result = inv_icm20602_mems_write_reg(s, MPUREG_I2C_SLV0_CTRL, 1, &d);
	if (result)
		return result;
	/* SLV1_CTRL */
	result = inv_icm20602_mems_read_reg(s, MPUREG_I2C_SLV1_CTRL, 1, &slv_ctrl[1]);
	if (result)
		return result;
	d = 0;
	result = inv_icm20602_mems_write_reg(s, MPUREG_I2C_SLV1_CTRL, 1, &d);
	if (result)
		return result;

	if (HW_AK09911 == s->secondary_state.compass_id)
		mode = REG_AK09911_CNTL2;
	else if (HW_AK09912 == s->secondary_state.compass_id)
		mode = REG_AK09912_CNTL2;
	else
		mode = REG_AKM_MODE;

	/* set to power down mode */
	result = inv_icm20602_write_secondary(s, 0, addr, mode, DATA_AKM_MODE_PD);
	if (result)
		goto AKM_fail;

	/* write 1 to ASTC register */
	if ((HW_AK09911 != s->secondary_state.compass_id) &&
		(HW_AK09912 != s->secondary_state.compass_id)) {
		result = inv_icm20602_write_secondary(s, 0, addr, REG_AKM_ST_CTRL, DATA_AKM_SELF_TEST);
		if (result)
			goto AKM_fail;
	}

	/* set self test mode */
	if (HW_AK09911 == s->secondary_state.compass_id)
		result = inv_icm20602_write_secondary(s, 0, addr, mode, DATA_AK09911_MODE_ST);
	else if (HW_AK09912 == s->secondary_state.compass_id)
		result = inv_icm20602_write_secondary(s, 0, addr, mode, DATA_AK09912_MODE_ST);
	else
		result = inv_icm20602_write_secondary(s, 0, addr, mode,	DATA_AKM_MODE_ST);

	if (result)
		goto AKM_fail;

	counter = DEF_ST_COMPASS_TRY_TIMES;

	while (counter > 0) {
		inv_icm20602_sleep(15);

		if (HW_AK09911 == s->secondary_state.compass_id)
			result = inv_icm20602_execute_read_secondary(s, 0, addr, REG_AK09911_STATUS1, 1, data);
		else if (HW_AK09912 == s->secondary_state.compass_id)
			result = inv_icm20602_execute_read_secondary(s, 0, addr, REG_AK09912_STATUS1, 1, data);
		else
			result = inv_icm20602_execute_read_secondary(s, 0, addr, REG_AKM_STATUS, 1, data);
		if (result)
			goto AKM_fail;
		if ((data[0] & DATA_AKM_DRDY) == 0)
			counter--;
		else
			counter = 0;
	}
	if ((data[0] & DATA_AKM_DRDY) == 0) {
		result = -1;
		goto AKM_fail;
	}
	if (HW_AK09911 == s->secondary_state.compass_id) {
		result = inv_icm20602_execute_read_secondary(s, 0, addr,REG_AK09911_MEASURE_DATA, BYTES_PER_SENSOR, data);
	} else if (HW_AK09912 == s->secondary_state.compass_id) {
		result = inv_icm20602_execute_read_secondary(s, 0, addr, REG_AK09912_MEASURE_DATA, BYTES_PER_SENSOR, data);
	} else {
		result = inv_icm20602_execute_read_secondary(s, 0, addr, REG_AKM_MEASURE_DATA, BYTES_PER_SENSOR, data);
	}
	if (result)
		goto AKM_fail;

	x = ((short)data[1])<<8|data[0];
	y = ((short)data[3])<<8|data[2];
	z = ((short)data[5])<<8|data[4];

	/* apply ASA if available */
	if (s->secondary_state.compass_sens[0] &&
	    s->secondary_state.compass_sens[1] &&
		s->secondary_state.compass_sens[2]) {
		if (HW_AK09911 == s->secondary_state.compass_id)
			shift = 7;
		else
			shift = 8;
		x = ((x * (sens[0] + 128)) >> shift);
		y = ((y * (sens[1] + 128)) >> shift);
		z = ((z * (sens[2] + 128)) >> shift);
		if (HW_AK8963 == s->secondary_state.compass_id) {
			result = inv_icm20602_execute_read_secondary(s, 0, addr, REG_AKM8963_CNTL1, 1, &cntl);
			if (result)
				goto AKM_fail;
			if (0 == (cntl & DATA_AKM8963_BIT)) {
				x <<= DEF_ST_COMPASS_8963_SHIFT;
				y <<= DEF_ST_COMPASS_8963_SHIFT;
				z <<= DEF_ST_COMPASS_8963_SHIFT;
			}
		}
	}

	result = -1;
	if (x > s->secondary_state.st_upper[0] || x <  s->secondary_state.st_lower[0])
		goto AKM_fail;
	if (y >  s->secondary_state.st_upper[1] || y <  s->secondary_state.st_lower[1])
		goto AKM_fail;
	if (z >  s->secondary_state.st_upper[2] || z <  s->secondary_state.st_lower[2])
		goto AKM_fail;
	result = 0;
AKM_fail:
	/*write 0 to ASTC register */
	if ((HW_AK09911 != s->secondary_state.compass_id) &&
		(HW_AK09912 != s->secondary_state.compass_id)) {
		result |= inv_icm20602_write_secondary(s, 0, addr, REG_AKM_ST_CTRL, 0);
	}
	/*set to power down mode */
	result |= inv_icm20602_write_secondary(s, 0, addr, mode, DATA_AKM_MODE_PD);

	return result;
}

#if 0 // TODO: only for ak8963, can be removed?
/*
 *  inv_write_akm_scale() - Configure the akm scale range.
 */
int inv_icm20602_write_akm_scale(struct inv_icm20602 * s, int data)
{
	unsigned char d, en;
	int result;
    
	if (HW_AK8963 != s->secondary_state.compass_id)
		return 0;
	en = !!data;
	if (scale == en)
		return 0;
	d = (DATA_AKM_MODE_SM | (en << DATA_AKM8963_SCALE_SHIFT));

	result = inv_icm20602_mems_write_reg(s, MPUREG_I2C_SLV1_DO, 1, &d);
	if (result)
		return result;

	scale = en;
    
	return 0;
}
#endif

/*
 *  inv_read_akm_scale() - show AKM scale.
 */
int inv_icm20602_read_akm_scale(struct inv_icm20602 * s, int *scale)
{
	if (HW_AK8975 == s->secondary_state.compass_id)
		*scale = DATA_AKM8975_SCALE;
	else if (HW_AK8972 == s->secondary_state.compass_id)
		*scale = DATA_AKM8972_SCALE;
	else if (HW_AK8963 == s->secondary_state.compass_id)
		if (*scale)
			*scale = DATA_AKM8963_SCALE1;
		else
			*scale = DATA_AKM8963_SCALE0;
	else if (HW_AK09911 == s->secondary_state.compass_id)
		*scale = DATA_AK09911_SCALE;
	else if (HW_AK09912 == s->secondary_state.compass_id)
		*scale = DATA_AK09912_SCALE;
	else
		return -1;

	return 0;
}

int inv_icm20602_suspend_akm(struct inv_icm20602 * s)
{
	int result;

	if (!s->secondary_state.secondary_resume_state)
		return 0;

	/* Switch off AUX I2C to reconfigure SLV for compass atomically */
	result = inv_icm20602_secondary_disable_i2c(s);

	/* slave 0 is disabled */
	result |= inv_icm20602_secondary_stop_channel(s, COMPASS_I2C_SLV_READ);
	/* slave 1 is disabled */
	result |= inv_icm20602_secondary_stop_channel(s, COMPASS_I2C_SLV_WRITE);
	if (result)
		return result;

	/* Enable AUX I2C again if there are other devices in use */
	// TODO: Check other devices on AUX I2C
	//result |= inv_icm20602_secondary_enable_i2c(s);

	s->secondary_state.secondary_resume_state = 0;

	return result;
}

int inv_icm20602_resume_akm(struct inv_icm20602 * s)
{
	int result;
	uint8_t reg_addr, bytes;
	unsigned char lDataToWrite;
	uint8_t scale;

	if (s->secondary_state.secondary_resume_state)
		return 0;

	/* slave 0 is used to read data from compass */
	/*read mode */

	/* AKM status register address is 1 */
	if (HW_AK09911 == s->secondary_state.compass_id) {
		reg_addr = REG_AK09911_STATUS1;
		bytes = DATA_AKM_99_BYTES_DMP - 1;
	} else if (HW_AK09912 == s->secondary_state.compass_id) {
		reg_addr = REG_AK09912_STATUS1;
		bytes = DATA_AKM_99_BYTES_DMP - 1;
	} else {
		reg_addr = REG_AKM_STATUS;
		bytes = DATA_AKM_89_BYTES_DMP - 1;
	}

	result = inv_icm20602_read_secondary(s, COMPASS_I2C_SLV_READ, s->secondary_state.compass_i2c_addr, reg_addr, bytes);
	if (result)
		return result;

	/* slave 1 is used to write one-shot accquisition configuration to compass */
	/* output data for slave 1 is fixed, single measure mode */
	scale = 1;
	if (HW_AK8975 == s->secondary_state.compass_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	} else if (HW_AK8972 == s->secondary_state.compass_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	} else if (HW_AK8963 == s->secondary_state.compass_id) {
		lDataToWrite = DATA_AKM_MODE_SM |
			(scale << DATA_AKM8963_SCALE_SHIFT);
	} else if (HW_AK09911 == s->secondary_state.compass_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	} else if (HW_AK09912 == s->secondary_state.compass_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	} else {
		return -1;
	}

	result = inv_icm20602_write_secondary(s, COMPASS_I2C_SLV_WRITE, s->secondary_state.compass_i2c_addr, s->secondary_state.mode_reg_addr, lDataToWrite);
	if (result)
		return result;

	result |= inv_icm20602_secondary_enable_i2c(s);

	s->secondary_state.secondary_resume_state = 1;

	return result;
}

char inv_icm20602_compass_getstate(struct inv_icm20602 * s)
{
	return s->secondary_state.secondary_resume_state;
}

int inv_icm20602_get_compass_data(struct inv_icm20602 * s,
		const unsigned char *packet, unsigned char *raw_compass)
{
	int shift, i;
	union {
		unsigned char c[2];
		short s;
	} data[3];
	//uint8_t st1, st2;

	(void)s;

	data[0].s = ((short)packet[2])<<8|packet[1];
	data[1].s = ((short)packet[4])<<8|packet[3];
	data[2].s = ((short)packet[6])<<8|packet[5];

	/* apply ASA if available */
	if (s->secondary_state.compass_sens[0] &&
		s->secondary_state.compass_sens[1] &&
		s->secondary_state.compass_sens[2]) {
		shift = 8;
		if (HW_AK09911 == s->secondary_state.compass_id)
			shift = 7;
		data[0].s = ((data[0].s * (s->secondary_state.compass_sens[0] + 128)) >> shift);
		data[1].s = ((data[1].s * (s->secondary_state.compass_sens[1] + 128)) >> shift);
		data[2].s = ((data[2].s * (s->secondary_state.compass_sens[2] + 128)) >> shift);
	}

	/* from little endian to big endian */
	for (i = 0; i < 3; i++) {
		raw_compass[i*2] = data[i].c[1];
		raw_compass[i*2+1] = data[i].c[0];
	}

	// TODO: any action based on ST1/ST2 ?
#if 0
	st1 = packet[0];
	if ((HW_AK09911 == s->secondary_state.compass_id) ||
		(HW_AK09912 == s->secondary_state.compass_id))
		st2 = packet[8];
	else
		st2 = packet[7];
#endif

	return 0;
}

int inv_icm20602_get_compass_bytes(struct inv_icm20602 * s)
{
	int bytes = 0;

	/* AKM status register address is 1 */
	if (HW_AK09911 == s->secondary_state.compass_id) {
		bytes = DATA_AKM_99_BYTES_DMP - 1;
	} else if (HW_AK09912 == s->secondary_state.compass_id) {
		bytes = DATA_AKM_99_BYTES_DMP - 1;
	} else {
		bytes = DATA_AKM_89_BYTES_DMP - 1;
	}
	return bytes;
}
#endif
