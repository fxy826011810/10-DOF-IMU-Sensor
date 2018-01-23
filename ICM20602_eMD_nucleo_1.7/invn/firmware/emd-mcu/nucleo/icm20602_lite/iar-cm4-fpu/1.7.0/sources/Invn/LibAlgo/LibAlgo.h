/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
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

#include <stdint.h>
#include "Invn/InvExport.h"

/**
* \brief Predefined use cases
*/
enum algo_mode {
	HMD_VR_MODE,	//!< Longer (and less sensitive) gyro calibration, discard small head movements
	MOBILE_MODE,	//!< Regular mode, calibration is more sensitive to small movements
	ALGO_MODES
};

/*------ Defines -----------------------------------------------------------------*/
/*
 * These below defines are copied from algorithm configuration files
 * to avoid to much headers dependancy to install in package for few configuration used
 * /!\ Warning /!\ on algortihm version update, these defines may change!
 */
 
#define ALGO_INVN_CALIBRATION_ACGO_CONFIG_SIZE    512
#define ALGO_INVN_CALIBRATION_CCGO_CONFIG_SIZE    512
#define ALGO_INVN_CALIBRATION_GYR_CAL_FXP_SIZE    256
#define ALGO_INVN_CALIBRATION_GYRO_BT_CONFIG_SIZE 560
#define ALGO_INVN_ORIENTATION_CONFIG_SIZE         624
#define ALGO_INVN_PREDICTIVEQUATERNION_CONFIG_SIZE 52
#define ALGO_INVN_GESTURE_CONFIG_SIZE             360
#define ALGO_INVN_AAR_CONFIG_SIZE                1620

 /* Parameters for Gyroscope calibration algorithm (VR/HMD) */
#define	ALGO_HMD_GYRO_CALIBRATION_SAMPLE_NUM_LOG2								8
#define	ALGO_HMD_GYRO_CALIBRATION_DURATION_US									2000000
#define	ALGO_HMD_LOW_NOISE_GYRO_CALIBRATION_DATA_DIFF_THRESHOLD					38
#define	ALGO_HMD_LOW_NOISE_GYRO_CALIBRATION_HIGH_ORDER_VARIANCE_THRESHOLD		50
 /* Parameters for Gyroscope calibration algorithm (Mobile) */
#define	ALGO_MOBILE_GYRO_CALIBRATION_SAMPLE_NUM_LOG2							6
#define	ALGO_MOBILE_GYRO_CALIBRATION_DURATION_US								500000
#define	ALGO_MOBILE_LOW_POWER_GYRO_CALIBRATION_DATA_DIFF_THRESHOLD				20
#define	ALGO_MOBILE_LOW_POWER_GYRO_CALIBRATION_HIGH_ORDER_VARIANCE_THRESHOLD	3
#define	ALGO_MOBILE_LOW_NOISE_GYRO_CALIBRATION_DATA_DIFF_THRESHOLD				8
#define	ALGO_MOBILE_LOW_NOISE_GYRO_CALIBRATION_HIGH_ORDER_VARIANCE_THRESHOLD	2

#define	ALGO_DEFAULT_GYRO_CALIBRATION_FNM_BIAS_REJECTION_THRESHOLD				21474836L

 /* Parameters for Gyroscope Bias Tracker */
#define ALGO_INVN_CALIBRATION_GYRO_BT_DEFAULT_SAMPLING 10000

 /* Parameters for Predictive Quaternion */
#define ALGO_INVN_PREDICTIVEQUATERNION_DEFAULT_METHOD_ENNUM 1
#define ALGO_INVN_PREDICTIVEQUATERNION_DEFAULT_PREDICTIVE_TIME_US 20000

/*** Calibration ***/

/**
* \brief Check if an HMD device is targeted.
* \param[in] opt Additional parameters, can be NULL. \n Fixed point format is \b Q0\b .
* \return true/false for HMD device target.
* \ingroup HMDDevice
*/
uint8_t INV_EXPORT Algo_InvnCalibration_isHMDDevice(void * opt);

/* AccelCalibrationGyroOptionalFxp */

/**
* \brief Initializes the calibration algorithm. this function should be called at the first start of sensor but should not be called after a switch off/on of the sensor since the accelerometer offset is not subject to change during off period.
* \param[in] data Address of the structure of size INVN_CALIBRATION_ACGO_CONFIG_SIZE to use internally. 
* \param[in] offset[3] initial offset [g]. fxp format is \b q25 \n If pointer is NULL or 0, offset will be set to { 0, 0, 0},
* \param[in] accuracy accuracy of initial offset (0 to 3).
* \param[in] acc_sampling_period sampling period [us] of the accelerometer sensor. If called with acc_sampling_period==0, the previous value of sampling period will be kept.
* \warning This method sets default parameters value, it will resets the algorithm and set accuracy to 0.
* \ingroup AccelCalibrationGyroOptional
*/
void INV_EXPORT Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_Init(void* data, int32_t accOffset[3], int32_t accuracy, uint32_t acc_sampling_period);

/**
* \brief This function allows to change accel sampling period on the fly without reseting the algorithm and so keeping accuracy level.
* \param[in] data Address of the structure of size INVN_CALIBRATION_ACGO_CONFIG_SIZE to use internally. 
* \param[in] acc_sampling_period sampling period [us] of the accelerometer sensor. If called with acc_sampling_period==0, the previous value of sampling period will be kept.
* \warning This method will keep offset and accuracy level.
* \ingroup AccelCalibrationGyroOptional
*/
void INV_EXPORT Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_SetSamplingPeriod(void *data, uint32_t acc_sampling_period);

/**
* \brief Gets accuracy level of current offset.
* \param[in] data Address of the structure of size INVN_CALIBRATION_ACGO_CONFIG_SIZE to use internally.
* \retval 0 Unreliable
* \retval 1 Low
* \retval 2 Medium
* \retval 3 High
* \ingroup AccelCalibrationGyroOptional
*/
uint8_t INV_EXPORT Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_GetAccuracy(void *data);

/**
* \brief Updates the algorithm with new accelerometer sensor data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_ACGO_CONFIG_SIZE to use internally.
* \param[in] acc_uncalib, uncalibrated accelerometer data [g]. fxp format is \b q25
* \param[in] acc_temperature, uncalibrated accelerometer temperature in °C*100
* \param[out] acc_offset[3] offset [g], coded in q25.
* \return 1 when new position has been taken into account,
* \return 2 when a new offset has been validated,
* \return 0 otherwise.
* \ingroup AccelCalibrationGyroOptional
*/
int32_t INV_EXPORT Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_UpdateAcc(void* data, const int32_t acc_uncalib[3], const int32_t acc_temperature, int32_t acc_offset[3]);

/**
* \brief Updates the algorithm with new gyrometer data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_ACGO_CONFIG_SIZE to use internally.
* \param[in] gyro_data[3] gyrometer measurment in [rps]. fxp format is \b q16
* \param[in] opt Additional parameters, can be NULL. \n Fixed point format is \b Q0\b .
* \param[in] dt time since last sample in [us].
* \param[in] gyr_accuracy accuracy of current gyrometer measurment (0 to 3).
* \warning One must not use both InvnCalibration_AccelCalibrationGyroOptionalFxp_updateGyr and InvnCalibration_AccelCalibrationGyroOptionalFxp_updateQuat, they are both used to update the gyro integration used in the algorithm. If the gyro integrated quaternion has been computed previously one should rather use RlsCalibrationFxp_UpdateQuat.
* \sa InvnCalibration_AccelCalibrationGyroOptionalFxp_UpdateQuat.
* \ingroup AccelCalibrationGyroOptional
*/
void INV_EXPORT Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_UpdateGyr(void* data, const int32_t gyr[3], const uint32_t dt_us, int32_t gyr_accuracy, void *opt);

/* GyroCalibration */

/**
* \brief Initializes the calibration algorithm.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally. 
* \param[in] gyro_bias initial bias, fxp format is \b 2^30 = 2000dps. \n If pointer is NULL or 0, bias will be set to { 0, 0, 0}.
* \param[in] accuracy accuracy level of current bias.
* \warning This method sets default parameters value.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_Init(void* data, int32_t gyro_bias[3], int32_t accuracy);

/**
* \brief Reset the calibration algorithm state. 
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally.
* \warning This method does not reset parameter, bias or accuracy values.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_Reset(void* data);

/**
* \brief Sets parameters with a user-friendly interface.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally. 
* \param[in] rate Decimation factor. Number of times we skip update calls, e.g. when rate=3, algorithm runs 1 over 3 times. Default value is 0. 
* \param[in] algo_mode Configure set of param according to a predefined use case (\sa algo_mode)
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_SetUserParam(void *data, int32_t rate, enum algo_mode algo_mode);

/**
* \brief Sets custom parameters.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally. 
* \param[in] duration_us The minimum time [us] required to calibrate gyroscope bias. 
* \param[in] sample_number_log2 Log2 of number of samples to compute the bias, e.g. when sample_number_log2 = 6, number of samples is 2^6=64 samples. Default value is 6.
* \param[in] data_difference_threshold Threshold that detect motion. Restart algorithm when the difference 2 consequetive samples exceed the threshold. Default value is 58. 
* \param[in] high_order_variance_threshold Threshold that detect motion. Restart algorithm when the high order variance exceed the threshold. Default value is 0x28000.
* \param[in] bias_rejection_threshold Threshold that detect motion. Reject bias higher than the threshold. Default value is 40 dps => 40*(2^15/2000)*2^15.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_SetParam(void *data,
								int32_t		duration_us,
								int32_t		sample_number_log2,
								int32_t		data_difference_threshold,
								int32_t		high_order_variance_threshold,
								int32_t		bias_rejection_threshold);


/**
* \brief Gets calibration algorithm parameters.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally. 
* \param[in] duration_us The minimum time required to calibrate gyroscope bias. 
* \param[in] sample_number_log2 Log2 of number of samples to compute the bias, e.g. when sample_number_log2 = 6, number of samples is 2^6=64 samples. Default value is 6.
* \param[in] data_difference_threshold Threshold that detect motion. Restart algorithm when the difference 2 consequetive samples exceed the threshold. Default value is 58. 
* \param[in] high_order_variance_threshold Threshold that detect motion. Restart algorithm when the high order variance exceed the threshold. Default value is 0x28000.
* \param[in] bias_rejection_threshold Threshold that detect motion. Reject bias higher than the threshold. Default value is 40 dps => 40*(2^15/2000)*2^15.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_GetParam(void *data,
							int32_t		*duration_us,
							int32_t		*sample_number_log2,
							int32_t		*data_difference_threshold,
							int32_t		*high_order_variance_threshold,
							int32_t		*bias_rejection_threshold);


/**
* \brief Set gyrometer calibration time sampling period.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally. 
* \param[in] sampling_period_us sampling period in us. 
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_SetSamplingPeriod(void *data, uint32_t	sampling_period_us);

/**
* \brief Gets accuracy level of current bias.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally.
* \retval 0 Unreliable
* \retval 1 Low
* \retval 2 Medium
* \retval 3 High
* \ingroup GyroCalibration
*/
uint8_t INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_GetAccuracy(void* data);

/**
* \brief Gets calibrated data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally.
* \param[out] gyro_calibrated[3] calibrated data, fxp format is \b 2^30 = 2000dps.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_GetCalibrated(void* data, int32_t gyro_calibrated[3]);

/**
* \brief Gets uncalibrated data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally.
* \param[out] gyro_uncalibrated[3] calibrated data, fxp format is \b 2^30 = 2000dps.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_GetUncalibrated(void* data, int32_t gyro_uncalibrated[3]);

/**
* \brief Sets bias data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally.
* \param[out] gyro_bias[3] gyro bias, fxp format is \b 2^30 = 2000dps.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_SetBias(void *data, const int32_t	bias[3]);

/**
* \brief Sets accuracy level.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally.
* \param[out] accuracy gyro bias accuracy level, accuracy values are 0: unreliable, 1: low, 2:medium, 3:High.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_SetAccuracy(void *data, const int32_t	accuracy);

/**
* \brief Gets uncalibrated data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally.
* \param[out] gyro_bias[3] gyro bias, fxp format is \b 2^30 = 2000dps.
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_GetBias(void* data, int32_t gyro_bias[3]);

/**
* \brief Updates the algorithm with new accelerometer sensor data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYR_CAL_FXP_SIZE to use internally.
* \param[in] gyro_raw[3] raw gyroscope data, fxp format is \b 2^15 = 2000dps.
* \param[in] gyro_temperature, gyroscope temperature in °C*100
* \ingroup GyroCalibration
*/
void INV_EXPORT Algo_InvnCalibration_GyroCalibrationFxp_UpdateGyr(void* data, const int32_t gyro_raw[3], const int32_t gyro_temperature);

/* BiasTracker */

/**
* \brief Initializes the bias tracker algorithm. This function should be called at the first start of sensor..
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYROBT_CONFIG_SIZE to use internally. 
* \warning This method sets default parameters value, it will resets the algorithm and set offset and accuracy to 0.
* \ingroup GyroBiasTracker
*/
void INV_EXPORT Algo_InvnCalibration_GyroBiasTrackerFxp_Init(void* data);
	
/**
* \brief Resets the states of the bias tracker algorithm.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYROBT_CONFIG_SIZE to use internally. 
* \warning This function will keep current algo parameters.
* \ingroup GyroBiasTracker
*/
void INV_EXPORT Algo_InvnCalibration_GyroBiasTrackerFxp_ResetStates(void* data);
	
/**
* \brief This function allows to change gyroscope sampling period on the fly without reseting the algorithm and so keeping accuracy level.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYROBT_CONFIG_SIZE to use internally. 
* \param[in] gyro_sampling_period sampling period [us] of the gyrometer sensor.
* \retval -1 if sampling period is >10000us. Previous sampling period is kept.
* \retval 0 otherwise.
* \warning The maximum sampling period for bias tracker is 10000us. This method will keep current offset and accuracy level.
* \ingroup GyroBiasTracker
*/
int8_t INV_EXPORT Algo_InvnCalibration_GyroBiasTrackerFxp_SetGyrSamplingPeriod(void* data, uint32_t gyro_sampling_period);

/**
* \brief This function allows to set a bias with an associated accuracy level.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYROBT_CONFIG_SIZE to use internally.
* \param[in] gyr_offset[3] gyrometer offset in [1/2000 dps], fxp format is \b q30.
* \param[in] accuracy accuracy level of bias (0 to 3).
* \ingroup GyroBiasTracker
*/
void INV_EXPORT Algo_InvnCalibration_GyroBiasTrackerFxp_SetBias(void* data, int32_t gyro_offset[3], int8_t accuracy);

/**
* \brief Gets accuracy level of current offset.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYROBT_CONFIG_SIZE to use internally.
* \retval 0 Unreliable
* \retval 1 Low
* \retval 2 Medium
* \retval 3 High
* \ingroup GyroBiasTracker
*/
int8_t INV_EXPORT Algo_InvnCalibration_GyroBiasTrackerFxp_GetAccuracy(void* data);
	
/**
* \brief Updates the algorithm with new magnetometer sensor data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYROBT_CONFIG_SIZE to use internally.
* \param[in] mag_calib[3], calibrated magnetometer data [uT]. fxp format is \b q16
* \param[out] mag_accuracy_flag accuracy flag of calibrated magnetometer.
* \ingroup GyroBiasTracker
*/
void INV_EXPORT Algo_InvnCalibration_GyroBiasTrackerFxp_UpdateMag(void* data, const int32_t mag_calib[3], const int8_t mag_accuracy_flag);
	
/**
* \brief Updates the algorithm with new accelerometer sensor data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYROBT_CONFIG_SIZE to use internally.
* \param[in] acc_calib[3], calibrated accelerometer data [g]. fxp format is \b q25.
* \param[out] acc_accuracy_flag accuracy flag of calibrated accelerometer (0 to 3).
* \ingroup GyroBiasTracker
*/
void INV_EXPORT Algo_InvnCalibration_GyroBiasTrackerFxp_UpdateAcc(void* data, const int32_t acc_calib[3], const int8_t acc_accuracy_flag);

/**
* \brief Updates the algorithm with new gyrometer data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_GYROBT_CONFIG_SIZE to use internally.
* \param[in] gyr_uncalib[3] uncalibrated gyrometer measurment in [1/2000 dps], fxp format is \b q30.
* \param[in] gyr_offset_fnm[3] \b (optional) gyrometer offset computed by FNM algorithm in [1/2000 dps], fxp format is \b q30. If the update function is called with a NULL pointer, this input will be ignored.
* \param[in] gyr_accuracy_flag_fnm \b (optional) accuracy flag (0 to 3) of FNM offset. If gyr_offset_fnm[3] is a NULL pointer, this input will be ignored.
* \param[in] gyr_temp \b (optional) temperature of gyrometer sensor in [°C*100]. If the update is called with a constant value of 0 this input will have no effect.
* \param[in] gyr_accuracy accuracy of current gyrometer measurment (0 to 3).
* \param[out] gyr_offset[3] gyrometer offset in [1/2000 dps], fxp format is \b q30.
* \return 1 when new sample has been taken into account,
* \return 2 when a new offset has been validated,
* \return 0 otherwise.
* \ingroup GyroBiasTracker
*/
int32_t INV_EXPORT Algo_InvnCalibration_GyroBiasTrackerFxp_UpdateGyr(void* data, const int32_t gyr_uncalib[3], const int32_t gyr_offset_fnm[3], const int8_t gyr_accuracy_flag_fnm, const int32_t gyr_temp, int32_t gyr_offset[3]);


/* CompassCalibrationGyroOptionalFxp */

/**
* \brief Initializes the calibration algorithm. this function should be called at the first start of sensor and also after a switch off/on of the sensor since the mag offset has potentially changed during off period.
* \param[in] data Address of the structure of size INVN_CALIBRATION_CCGO_CONFIG_SIZE to use internally. 
* \param[in] offset[3] initial offset [uT]. fxp format is \b q16 \n If pointer is NULL or 0, offset will be set to { 0, 0, 0},
* \param[in] accuracy accuracy of initial offset (0 to 3).
* \param[in] mag_sampling_period sampling period [us] of the magnetometer sensor. If called with mag_sampling_period==0, the previous value of sampling period will be kept.
* \warning This method sets default parameters value, it will resets the algorithm and set accuracy to 0.
* \ingroup CompassCalibrationGyroOptional
*/
void INV_EXPORT Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_Init(void* data, int32_t magOffset[3], int32_t accuracy, uint32_t mag_sampling_period);

/**
* \brief This function allows to change compass sampling period on the fly without reseting the algorithm and so keeping accuracy level.
* \param[in] data Address of the structure of size INVN_CALIBRATION_CCGO_CONFIG_SIZE to use internally. 
* \param[in] mag_sampling_period sampling period [us] of the magnetometer sensor. If called with mag_sampling_period==0, the previous value of sampling period will be kept.
* \warning This method will keep offset and accuracy level.
* \ingroup CompassCalibrationGyroOptional
*/
void INV_EXPORT Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_SetSamplingPeriod(void* data, uint32_t mag_sampling_period);

/**
* \brief Gets accuracy level of current offset.
* \param[in] data Address of the structure of size INVN_CALIBRATION_CCGO_CONFIG_SIZE to use internally.
* \retval 0 Unreliable
* \retval 1 Low
* \retval 2 Medium
* \retval 3 High
* \ingroup CompassCalibrationGyroOptional
*/
uint8_t INV_EXPORT Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_GetAccuracy(void* data);

/**
* \brief Gets the norm of the local measured field.
* \param[in] data Address of the structure of size INVN_CALIBRATION_CCGO_CONFIG_SIZE to use internally.
* \retval field_norm magnitude of local magnetic field [uT] corresponding to the current offset solution. fxp format is \b q16
* \retval 0 if local magnetic field norm can't be computed (no offset computed yet).
* \ingroup CompassCalibrationGyroOptional
*/
uint32_t INV_EXPORT Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_GetFieldNorm(void* data);

/**
* \brief Updates the algorithm with new magnetometer sensor data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_CCGO_CONFIG_SIZE to use internally.
* \param[in] mag_uncalib, uncalibrated magnetometer data [uT]. fxp format is \b q16
* \param[out] mag_offset[3] offset [uT], coded in q16.
* \return 1 when new position has been taken into account,
* \return 2 when a new offset has been validated,
* \return 0 otherwise.
* \ingroup CompassCalibrationGyroOptional
*/
int32_t INV_EXPORT Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_UpdateMag(void* data, const int32_t mag_uncalib[3], int32_t mag_offset[3]);

/**
* \brief Updates the algorithm with new gyrometer data.
* \param[in] data Address of the structure of size INVN_CALIBRATION_CCGO_CONFIG_SIZE to use internally.
* \param[in] gyro_data[3] gyrometer measurment in [rps]. fxp format is \b q16
* \param[in] dt time since last sample in [us].
* \param[in] opt Additional parameters, can be NULL. \n Fixed point format is \b Q0\b .
* \param[in] gyr_accuracy accuracy of current gyrometer measurment (0 to 3).
* \warning One must not use both InvnCalibration_CompassCalibrationGyroOptionalFxp_updateGyr and InvnCalibration_CompassCalibrationGyroOptionalFxp_updateQuat, they are both used to update the gyro integration used in the algorithm. If the gyro integrated quaternion has been computed previously one should rather use RlsCalibrationFxp_UpdateQuat.
* \sa InvnCalibration_CompassCalibrationGyroOptionalFxp_UpdateQuat.
* \ingroup CompassCalibrationGyroOptional
*/
void INV_EXPORT Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_UpdateGyr(void* data, const int32_t gyr[3], const uint32_t dt_us, const int32_t gyr_accuracy, void * opt);

/*** Orientation ***/
/* BodyToWorldFrameFxp */
/**
* \brief Initializes Orientation algorithm with default parameters and reset states.
* \param[in] data Address of the structure of size INVN_ORIENTATION_CONFIG_SIZE to use internally.
* \warning Default parameter for sampling period is 10000 us.
* \ingroup BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_Init(void* data);

/**
* \brief Reset Orientation algorithm states. 
* \param[in] data Address of the structure of size INVN_ORIENTATION_CONFIG_SIZE to use internally.
* \ingroup BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_ResetStates(void* data);

/** \brief  Enable orientation output.
	
	\param[in,out] data: Pointer to data structure. 
	\param[in] mask: select one or more orientation to enable. \n
	\t 1: select Geo-mag Rotation Vector (GmRV)
	\t 2: select Game Rotation Vector (GRV)
	\t 4: select Rotation Vector (RV)
	\t 7: to select all 
	
	\ingroup   BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_Enable(void* data, uint8_t mask);

/** \brief  Disable orientation output.
	
	\param[in,out] data: Pointer to data structure. 
	\param[in] mask: select one or more orientation to enable. \n
	\t 1: select Geo-mag Rotation Vector (GmRV)
	\t 2: select Game Rotation Vector (GRV)
	\t 4: select Rotation Vector (RV)
	\t 7: to select all 

	\ingroup   BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_Disable(void* data, uint8_t mask);

/** \brief  Enable Rotation Vector (RV) output.
	
	\param[in,out] data: Pointer to data structure. 
	
	\ingroup   BodyToWorldFrame
*/
static __inline void Algo_InvnOrientation_BodyToWorldFrameFxp_AGM_Enable(void* data)
{
	Algo_InvnOrientation_BodyToWorldFrameFxp_Enable(data, 0x04);
}

/** \brief  Disnable Rotation Vector (RV) output.
	
	\param[in,out] data: Pointer to data structure. 
	
	\ingroup   BodyToWorldFrame
*/
static __inline void Algo_InvnOrientation_BodyToWorldFrameFxp_AGM_Disable(void* data)
{
	Algo_InvnOrientation_BodyToWorldFrameFxp_Disable(data, 0x04);
}

/** \brief  Enable Game Rotation Vector (GRV) output.
	
	\param[in,out] data: Pointer to data structure. 
	
	\ingroup   BodyToWorldFrame
*/
static __inline void Algo_InvnOrientation_BodyToWorldFrameFxp_AG_Enable(void* data)
{
	Algo_InvnOrientation_BodyToWorldFrameFxp_Enable(data, 0x02);
}

/** \brief  Disable Game Rotation Vector (GRV) output.
	
	\param[in,out] data: Pointer to data structure. 
	
	\ingroup   BodyToWorldFrame
*/
static __inline void Algo_InvnOrientation_BodyToWorldFrameFxp_AG_Disable(void* data)
{
	Algo_InvnOrientation_BodyToWorldFrameFxp_Disable(data, 0x02);
}

/** \brief  Enable Geo-mag Rotation Vector (GmRV) output.
	
	\param[in,out] data: Pointer to data structure. 
	
	\ingroup   BodyToWorldFrame
*/
static __inline void Algo_InvnOrientation_BodyToWorldFrameFxp_AM_Enable(void* data)
{
	Algo_InvnOrientation_BodyToWorldFrameFxp_Enable(data, 0x01);
}

/** \brief  Disable Geo-mag Rotation Vector (GmRV) output.
	
	\param[in,out] data: Pointer to data structure. 
	
	\ingroup   BodyToWorldFrame
*/
static __inline void Algo_InvnOrientation_BodyToWorldFrameFxp_AM_Disable(void* data)
{
	Algo_InvnOrientation_BodyToWorldFrameFxp_Disable(data, 0x01);
}

/** \brief  Set accelerometer time stamp and related parameters.	
	\param[in,out] data: Pointer to data structure. 
	\param[in] dt_us: sampling period in micro second.
	\warning Minimum accelerometer frequency is 10Hz.

    \ingroup BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_SetAccSamplingPeriod(void* data, 
		int32_t			dt_us,
		void 			*opt);

/** \brief  Set magnetometer custom parameters.	
	\param[in,out] param Pointer to data structure. 
	\param[in] anomaly_rejection anomaly rejection percentage 0 to 100, (anomaly_rejection = percentage * 2^30/100).
	\param[in] thresh_yaw_stop_convergence threshold to stop RV convergence when error is below this threshold, (thresh_yaw_stop_convergence = threshold_rad * 2^30). To disactivate this feature set threshold = -1.
	\param[in] thresh_gyr_stop_convergence threshold to stop RV convergence when gyro is below this threshold, (thresh_gyr_stop_convergence = threshold_dps * 2^30/2000). To disactivate this feature set threshold = -1.
	\param[in] thresh_yaw_smooth_convergence factor [Q16] to smooth RV yaw convergence. The correction speed will be scaled to remain lower than threshold * gyro, (suggested value threshold = 0.01*2^16 = 650). To disactivate this feature set threshold = -1.
	\warning Minimum magnetometer frequency is 10Hz.

    \ingroup BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_SetMagCustomParams(void* data, 
		int32_t			anomaly_rejection,
		int32_t			thresh_yaw_stop_convergence,
		int32_t			thresh_gyr_stop_convergence,
		int32_t			thresh_yaw_smooth_convergence);

/** \brief  Set magnetometer time stamp and related parameters.	
	\param[in,out] param Pointer to data structure. 
	\param[in] dt_us sampling period in micro second.
	\warning Minimum magnetometer frequency is 10Hz.

    \ingroup BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_SetMagSamplingPeriod(void* data, 
											int32_t			dt_us);

/** \brief  Set gyroscope time stamp and related parameters.	
	\param[in,out] data: Pointer to data structure. 
	\param[in] dt_us: sampling period in micro second.
	\param[in] opt Additional parameters, can be NULL.
	\warning Minimum gyroscope frequency is 50Hz.

	\ingroup BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_SetGyrSamplingPeriod(void* data, 
											int32_t			dt_us,
											void			*opt);

/** \brief  Update orientation using accelerometer data.
	
	\param[in,out] data: Pointer to data structure. 
	\param[in] accelerometer: accelerometer data. Units are in [G] fixed point format is \b Q25\b (1G: 33554432). \n. 
	\param[in] acc_accuracy: accelerometer accuracy (accuracy values: 0 Unreliable, 1 Low, 2 Medium, 3 High). \n. 

	\ingroup   BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateAcc(void* data, const int32_t accelerometer[3], int32_t acc_accuracy);

/** \brief  Update orientation using gyroscope data.
	
	\param[in,out] data: Pointer to data structure. 
	\param[in] gyroscope: gyroscope data. Units are in [dps] fixed point format is \b Q30/2000dps\b (2000 dps: 1073741824). \n. 
	\param[in] gyr_accuracy: gyroscope accuracy (accuracy values: 0 Unreliable, 1 Low, 2 Medium, 3 High). \n. 

	\ingroup   BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateGyr(void* data, const int32_t gyroscope[3], int32_t gyr_accuracy);

/** \brief  Update orientation using magnetometer data.
	
	\param[in,out] data: Pointer to data structure. 
	\param[in] magnetometer: magnetometer data. Units are in [uT] fixed point format is \b Q16\b \n. 
	\param[in] mag_accuracy: magnetometer accuracy (accuracy values: 0 Unreliable, 1 Low, 2 Medium, 3 High). \n. 
	\param[in] local_mag_field_norm: Norm of local magnetic field. Units are in [uT] fixed point format is \b Q16dps\b \n.

	\ingroup   BodyToWorldFrame
*/
void INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateMag(void* data, const int32_t magnetometer[3], int32_t mag_accuracy, const int32_t local_mag_field_norm);

/** \brief  Get game rotation vector (GRV) and accuracy. This orientation uses acclerometer and gyroscope data.
	
	\param[in,out] data: Pointer to data structure. 
	\param[out] quaternion_acc_gyr: quaternion. Fixed point format is \b Q30 \b. \n. 
	\return accuracy: quaternion accuracy (accuracy values: 0 Unreliable, 1 Low, 2 Medium, 3 High). \n. 

	\ingroup   BodyToWorldFrame
*/
int32_t INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_GetGameRotationVector(void* data, int32_t quaternion_acc_gyr[4]);

/** \brief  Get rotation vector (RV) and accuracy. This orientation uses acclerometer, magnetometer and gyroscope data.
	
	\param[in,out] data: Pointer to data structure. 
	\param[out] quaternion_acc_gyr_mag: quaternion. Fixed point format is \b Q30 \b. \n. 
	\return accuracy: quaternion accuracy. Units are in [rad]. Fixed point format is \b Q27 \b (1rad: 134217728). \n. 

	\ingroup   BodyToWorldFrame
*/
int32_t INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_GetRotationVector(void* data, int32_t quaternion_acc_gyr_mag[4]);

/** \brief  Get rotation vector (RV) and accuracy. This orientation uses acclerometer, magnetometer and gyroscope data.
	
	\param[in,out] data: Pointer to data structure. 
	\param[out] quaternion_acc_mag: quaternion. Fixed point format is \b Q30 \b. \n. 
	\return accuracy: quaternion accuracy. Units are in [rad]. Fixed point format is \b Q27 \b (1rad: 134217728). \n. 

	\ingroup   BodyToWorldFrame
*/
int32_t INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_GetGeoMagRotationVector(void* data, int32_t quaternion_acc_mag[4]);

/** \brief  Get gyroscope quaternion integration (LPQ) and accuracy. This orientation uses gyroscope data.
	
	\param[in,out] data: Pointer to data structure. 
	\param[out] quaternion_acc_gyr: quaternion data. Fixed point format is \b Q30 \b. \n. 
	\return accuracy: quaternion accuracy (accuracy values: 0 Unreliable, 1 Low, 2 Medium, 3 High). \n. 

	\ingroup   BodyToWorldFrame
*/
int32_t INV_EXPORT Algo_InvnOrientation_BodyToWorldFrameFxp_GetGyrQuaternion(void* data, int32_t quaternion_gyr[4]);

    
/* PredictiveQuaternionFxp */
/**
* \brief Initializes predictive quaternion algorithm with default parameters and reset states.
* \param[in] data Address of the structure of size INVN_ORIENTATION_CONFIG_SIZE to use internally.
* \warning Default parameter for sampling period is 1000 us.
* \warning Default parameter for method_ennum is 1.
* \warning Default parameter for predictive_time is 20000 us
* \ingroup PredictiveQuaternion
*/
void INV_EXPORT Algo_InvnOrientation_PredictiveQuaternionFxp_Init(void*data);

/**
* \brief Reset predictive quaternion algorithm states. 
* \param[in] data Address of the structure of size INVN_ORIENTATION_CONFIG_SIZE to use internally.
* \ingroup PredictiveQuaternion
*/
void INV_EXPORT Algo_InvnOrientation_PredictiveQuaternionFxp_ResetStates(void* data);

/**
* \brief Set predictive quaternion sampling period algorithm parameters. 
* \param[in] data Address of the structure of size INVN_ORIENTATION_CONFIG_SIZE to use internally.
* \param[in] sampling_period It represents the time between two gyrometer samples. Unit is [us]. \n Fixed point format is \b Q0\b .
* \return State of the parametrization. 
* \retval 0 No trouble.
* \retval 1 User provided a bad sampling_period parameter (<=0).
* \ingroup PredictiveQuaternion
*/
uint8_t INV_EXPORT Algo_InvnOrientation_PredictiveQuaternionFxp_SetSamplingPeriod(void* data, const int32_t sampling_period);


/**
* \brief Set predictive quaternion algorithm parameters. 
* \param[in] data Address of the structure of size INVN_ORIENTATION_CONFIG_SIZE to use internally.
* \param[in] method_ennum It select the used algorithm to predict quaternion, 0: No prediction, 1: Constant rate prediction.
* \param[in] predictive_time It represents the time between the present quaternion and futur quaternion.   Unit is [us]. \n Fixed point format is \b Q0\b
* \ Range should be [250 - 40000 us]
* \return State of the parametrization. 
* \retval 0 No trouble.
* \retval 1 User provided a bad set of parameter.
* \ingroup PredictiveQuaternion
*/
uint8_t INV_EXPORT Algo_InvnOrientation_PredictiveQuaternionFxp_SetParam(void* data
 			, const int8_t method_ennum
			, const int32_t predictive_time);


/**
* \brief Performs predictive quaternion computation.
* \param[in] data Address of the structure of size INVN_ORIENTATION_CONFIG_SIZE bytes to use internally.
* \param[in] calibrated_gyro Calibrated gyrometer. \n
Units are in [1/2000dps] and sensor scaled on 2000dps. \n
Fixed point format is \b Q30\b . \n 
Coordinates reference frame is Android. 
This input is \b not optionnal \b \n 
* \param[in] quaternion_input Quaternion of orientation in the present time \n 
It represents in Body frame to Earth frame transformation.\n 
Fixed point format is \b Q30\b . \n 
This input is \b not optionnal \b. \n 
* \param[out] quaternion_predictive Quaternion of orientation in futur time (defined by predictive_time). \n 
It represents in Body frame to Earth frame transformation in the futur.\n 
Fixed point format is \b Q30\b . \n 
* \param[in] opt Additionnal parameter, can be NULL.
* \return State of the computation performed. 
* \retval 0 No trouble
* \retval 1 No valid computation of predictive quaternion  
* \warning ODR is the same as the input. No decimation performed.
* \ingroup PredictiveQuaternion
*/
uint8_t INV_EXPORT Algo_InvnOrientation_PredictiveQuaternionFxp_Update(void* data
 			, const int32_t calibrated_gyro[3]
			, const int32_t quaternion_input[4]
	 		, int32_t quaternion_predictive[4]
			, void * opt);


/* FltAAR */

/** \defgroup AAR AAR
* \struct InvnAARInput : input data needed to process AAR algorithm
* \warning proximity, gyrometer and magnetometer sensor are optional, please refers to used configuration. 
* \param acc: calibrated Accelerometers (in g)
* \param timestampAcc: timestamp of acc sensor (us), used to decimate data during update function
* \param gyr: calibrated gyrometer (in rad/s), can be NULL pointer (if no gyrometer sensor). 
* \param mag: calibrated magnetometer (in uT/47), can be NULL pointer (if no magnetometer sensor).
* \param Ps: proximity sensor value (5 for not contact, 0 contact with the sensor, can be NULL pointer (if no proximity sensor) 
* \param magNewDataFlag: mag new data is available, equal 1 when a new magnetometer data is available, 0 when no new data is 
* available (if configuration algorithm included magnetometer sensor)
 */
typedef struct
{	float 	acc[3];
    int32_t timestampAcc;
    float 	gyr[3];
    float 	mag[3];
    int16_t Ps;
    uint8_t magNewDataFlag;
} InvnAARInput;

/** \defgroup AAR AAR
* \struct InvnAAROutput : output data computed by AAR algorithm
* \param stateBAC :detection BAC event (1:in transport, 2:walking, 4:running, 8:biking, 16:tilting, 32:still)
* (plus any combination of activity, ex: 24=16+8, tilting and biking) \n
* \param statesProb : an integer array containing probabilities (ranging between 0 and 1, and represented in q15) of the different BAC states. 
* The probabilities are output in the following order : still, walking, running, biking, in transport, walking in transport, running in transport
* \param stateSMD : detection SMD event (0: no movement, 1: movement)
* \param statePedometerCounter : detection Counter event()
* \param statePedometerEventStep : detection step event()
* \param statePedometerCadencyPDR : detection Cadency pdr event()
* \param sensorActivateMag: flag indicates when the algorithm needs data from magnetometer sensor (0: do not activate 
* magnetometer sensor, 1: activate magnetometer sensor)
* \param sensorActivateProx: flag indicates when the algorithm needs data from proximity sensor (0: do not activate 
* proximity sensor, 1: activate proximity sensor)
* \param ER1_y1: Enery of Accelerometer low pass filter [1-6.9Hz]
* \param stateTilt: 2-second window average gravity changing by at least 
* 35 degrees since the activation or the last event generated by the sensor (0: no tilt, 1: tilt)
 */
typedef struct
{
    uint8_t stateBAC;
    int16_t statesProb[7];
    int32_t stateSMD;
    int32_t statePedometerCounter;
    int8_t 	statePedometerEventStep;
    int32_t statePedometerCadencyPDR;
	uint8_t sensorActivateMag;
	uint8_t sensorActivateProx;
	int32_t ER1_y1;
    uint8_t stateTilt;
} InvnAAROutput;

/**
* \brief Initializes AAR algorithm with its default values. 
* \param[in] data : structure of size INVN_AAR_CONFIG_SIZE
* \ingroup AAR
*/
void INV_EXPORT Algo_InvnAAR_FltAAR_Init(void* data, const int32_t sampling_period);

/**
* \brief set the working frequency of AAR algorithm
* only 56.25Hz and 50Hz are supported 
* \param[in] data : structure of size INVN_AAR_CONFIG_SIZE
* \param[in] set_frequency : set value to select algorithm working frequency, acceptable value are 56.25Hz and 50Hz 
* (default value is 56.25Hz)
* \ingroup AAR
*/
void INV_EXPORT Algo_InvnAAR_FltAAR_SetFrequency(void* data, float set_frequency);

/**
* \brief Performs AAR detection. 
* \param[in] data: structure of size INVN_AAR_CONFIG_SIZE
* \param[in] input: structure InvnAARInput containing input data
* \param[out] output: structure InvnAAROutput containing output datadetection BAC event (1:in transport, 2:walking, 4:running, 8:biking, 16:tilting, 32:still)
* \return 0 no process data (due to decimation process)
* \return 1 process data (with no problem)
*  \warning internal algorithm nominal accelerometer input frequency is 56.25 Hz or 50Hz (can be set with InvnAAR_FltAAR_setFrequency function), 
*  i.e., samplingPeriod == 17777us or samplingPeriod == 20000 us. Otherwise, a decimator is used during the update function using timestamp. 
*  Will not work if input frequency is below 56.25Hz / 50Hz. 
* \warning AAR feature can be configured in 5 different ways
* \warning Default, only accelerometer sensor is used (low power mode)
* \warning Mi, INVN_CONFIG_AAR_USE_MI, accelerometer, magnetometer, proximity and SMD vibration mode 
* \warning AM, INVN_CONFIG_AAR_USE_AM, accelerometer, magnetometer sensor 
* \warning APROX, INVN_CONFIG_AAR_USE_APROX, accelerometer, proximity sensor 
* \warning AMPROX, INVN_CONFIG_AAR_USE_AMPROX, accelerometer, magnetometer, proximity sensor 
* \ingroup AAR
*/
uint8_t INV_EXPORT Algo_InvnAAR_FltAAR_Update(void* data,
		InvnAARInput* input,
		InvnAAROutput* output);


/* Gesture */
/**
* \brief Initializes Pick_up_L algorithm with default parameters and reset states.
* \param[in] data : structure of size INVN_GESTURE_CONFIG_SIZE
* \ingroup Pick_up_L
*/
void INV_EXPORT Algo_InvnGesture_PickUpLFxp_Init(void *data);

/**
* \brief Performs Pick_up_L detection.
* \param[in] data : structure of size INVN_GESTURE_CONFIG_SIZE
* \param[in] acc : calibrated accelerometers. Units are in g and axis respect "Movea" axis reference.
* \param[in] proximity : proximity sensor value, has to be 5 if no proximity sensor. 
* \param[out] detection : pick up detection event.
* \return 0.
* \warning Input frequency of accelerometers has to be 56.25Hz, i.e. a sampling period of 17778us.
* \ingroup Pick_up_L
*/
uint8_t INV_EXPORT Algo_InvnGesture_PickUpLFxp_Update(void *data, short acc[3], short proximity, uint8_t *detection);
