/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
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

#include <stdint.h>

/* InvenSense drivers and utils */
#include "Invn/Devices/Drivers/Icm20602/Icm20602.h"
#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Invn/Devices/SensorTypes.h"
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/EmbUtils/DataConverter.h"
#include "Invn/EmbUtils/RingBuffer.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"

/* InvenSense LibExport */
#include "Invn/LibAlgo/LibAlgo.h"

/* board driver */
#include "uart.h"
#include "delay.h"
#include "gpio.h"
#include "i2c_master.h"
#include "spi_master.h"
#include "timer.h"
#include "flash_linker.h"
#include "flash_manager.h"

/* std */
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

/******************************************************************************/
/* Example configuration                                                      */
/******************************************************************************/

/* 
 * Select communication between STNucleo and ICM20602 by setting 0/1 to one of the following defines
 */
#define SERIF_TYPE_SPI 1
#define SERIF_TYPE_I2C 0

/*
 * Set to 1 to use compass AK09911 connected to STNucleo I2C
 */
#define USE_AK09911_MAG                   1


/* FSR configurations */
static int32_t cfg_acc_fsr = 4000; /* +/- 4g */
static int32_t cfg_gyr_fsr = 2000; /* +/- 2000dps */

/* Sensitivity configurations */
#define ACC_SENSITIVITY (int32_t) ( ((cfg_acc_fsr/1000) * (1 << 16)) / INT16_MAX)
#define GYR_SENSITIVITY (int32_t) ( (cfg_gyr_fsr * (1 << 16) / INT16_MAX) )
#if USE_AK09911_MAG
	#define MAG_SENSITIVITY (int32_t) (39321) /* = 0.6f * (1 << 16) [uT/LSB] */
	#define RV_ANOMALIES_REJECTION             1073741824L // = 100% rejection = 2^30L
	#define RV_THRESHOLD_YAW_STOP_CONVERGENCE  -1L         // 18740330L = 1 deg * pi/180 * 2^30
	#define RV_THRESHOLD_GYR_STOP_CONVERGENCE  1073742L    // 1073742L = 2dps * 2^30 / 2000
	#define RV_THRESHOLD_YAW_SMOOTH_CONVERGENCE_DEFAULT 650L // 650L = 1% * 2^16
#endif

/*
 * Other...
 */
 
#if USE_AK09911_MAG
	#define AK_I2C_ADDR  0x0C /* I2C address for Ak09911 */
	#define FREQ_TIMER_END_CAPTURE  115 /* AK09911 datasheet says that worst case capture takes 8.5ms ~= 117Hz, so we use 115Hz for the extra margin */
	#define MIN_MAG_ODR_US  10000 /* AK09911 maximum frequency is 100Hz */
#endif 

#define ICM_I2C_ADDR     0x69 /* I2C slave address for ICM20602 */
static const uint8_t EXPECTED_WHOAMI[] = { 0x12, 0x11 };  /* WHOAMI value for ICM20602 or derivative */
#define DATA_ACCURACY_MASK  ((uint32_t)0x7)

/* factor to convert degree to radian expressed in q30 */
#define FACTOR_DPS_TO_RPS       (int32_t) 18740330 // ((PI / 180) * (1 << 30))

/* sensor ODR limit */
#define MIN_ODR_US        1000
#define MAX_ODR_US       20000
#define DEFAULT_ODR_US   20000

/*
 * Sanity checks
 */

#if (SERIF_TYPE_SPI && SERIF_TYPE_I2C) || !SERIF_TYPE_SPI && !SERIF_TYPE_I2C
	#error "You must choose between I2C or SPI for icm20602 device control"
#endif

/*
 * Overload UART id to swap between each other
 * Some instability (BSOD) and byte loss were observed with UART2 (going through ST-Link bridge) under Windows.
 * Using UART1 or UART6 with FTDI allows more reliable communication (with the downside of having to use two USB cables even
 * if we don't care about traces)
 */
#define LOG_UART_ID  UART2
#define MAIN_UART_ID UART1

/*
 * Overload TIMER id to swap between each other
 */
#define TIMEBASE_TIMER            TIMER2
#define DELAY_TIMER               TIMER3
#if USE_AK09911_MAG
	#define MAG_SAMPLING_TIMER    TIMER4
#endif

/* UART buffer size configurations */
#define UART_LOG_FIFO_SIZE  4096
#define UART_RX_FIFO_SIZE    256
#define UART_TX_FIFO_SIZE   4096

/******************************************************************************/

/*
 * Some memory to be used by the UART driver (4kB each FIFO)
 */
static uint8_t uart_log_buffer[UART_LOG_FIFO_SIZE];
static uint8_t uart_mainRx_buffer[UART_RX_FIFO_SIZE]; 
static uint8_t uart_mainTx_buffer[UART_TX_FIFO_SIZE]; 

/*
 * Handy pointer to UART DMA RX buffer
 */
volatile uart_dma_rx_buffer_t * uart_dma_rx_buffer;

/*
 * Flag set once a UART RX frame is received
 */
static volatile int irq_event_main_uart = 0;

/*
 * Just a handy variable to handle the icm20602 object
 */
static inv_icm20602_t icm_device;

/*
 * Flag set from icm20602 device irq handler
 */
static volatile int irq_from_device;

/*
 * Mask to keep track of enabled sensors
 */
static uint32_t enabled_sensor_mask = 0;

/*
 * Variable to keep track of the expected period common for all sensors
 */
static uint32_t period_us = DEFAULT_ODR_US /* 50Hz by default */;

/*
 * Variable to drop the first timestamp(s) after a sensor start catched by the interrupt line.
 * This is needed to be inline with the driver. The first data polled from the FIFO is always discard.
 */
static uint8_t timestamp_to_drop = 0;

/*
 * Variable keeping track of chip information
 */
static uint8_t chip_info[3];

/*
 * Variable keeping track of gyro-assisted calibration support
 */
static uint8_t hmd_features_supported = 0;

#if USE_AK09911_MAG
/*
 * Just a handy variable to handle the ak0991x object
 */
static inv_ak0991x_t ak_device;

/* 
 * Flag set for Ak09911 capture start
 */
static volatile int irq_start_mag_capture;

/* 
 * Buffer to keep track of the timestamp at Ak09911 end of capture
 */
static RINGBUFFER(mag_timestamp_buffer, 128, uint64_t);

/* 
 * Timer channels to trigger on AK09911 start/end capture
 */
static int mag_start_capture_channel = -1; /*reset to unusable*/
static int mag_end_capture_channel = -1; /*reset to unusable*/

/* 
 * Boolean to track ak09911 setup & whoami reading success or not, to address it afterwards or not
 */
static uint8_t ak09911_is_available;
#endif

/* 
 * Sensor identifier for control function
 */
enum sensor {
	SENSOR_RAW_ACC,
	SENSOR_RAW_GYR,
	SENSOR_ACC,
	SENSOR_GYR,
	SENSOR_UGYR,
	SENSOR_GRV,
	SENSOR_PREDQUAT,
	SENSOR_GRA,
	SENSOR_LINACC,
#if USE_AK09911_MAG
	SENSOR_RAW_MAG,
	SENSOR_MAG,
	SENSOR_UMAG,
	SENSOR_RV,
	SENSOR_GEORV,
#endif

	SENSOR_MAX
};

/*
 * Data Structures 
 */
static int32_t sRacc_data[3];
static int32_t sRgyro_data[3];
static int16_t sRtemp_data;
#if USE_AK09911_MAG
static int32_t sRmag_data[3];
#endif

static struct {
	int32_t acc_cal_q16[3];
	int32_t acc_bias_q16[3];
	uint8_t accuracy_flag;
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_ACGO_CONFIG_SIZE];
		float   flt; /* ensure correct memory alignment of the buffer */
	} C_buffer;
} sCalAcc;

static struct {
	int32_t gyro_cal_2000dps_q30[3];
	int32_t gyr_cal_q16[3];
	int32_t gyr_uncal_q16[3];
	int32_t gyr_bias_q16[3];
	uint8_t accuracy_flag;
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_GYR_CAL_FXP_SIZE];
		float   flt; /* ensure correct memory alignment of the buffer */
	} C_buffer;
} sCalGyr;

static struct {
	int32_t grv_quat_q30[4];
	int32_t gravity_q16[3];
	int32_t linearacc_q16[3];
	union {
		uint8_t buf[ALGO_INVN_ORIENTATION_CONFIG_SIZE];
		float flt; /* ensure proper alignement */
	} C_buffer;
} sGRV;

static struct {
	int32_t predgrv_quat_q30[4];
	union {
		uint8_t buf[ALGO_INVN_PREDICTIVEQUATERNION_CONFIG_SIZE];
		float flt; /* ensure proper alignement */
	} C_buffer;
} sPredGRV;

#if USE_AK09911_MAG
static struct {
	int32_t mag_cal_q16[3];
	int32_t mag_uncal_q16[3];
	int32_t mag_bias_q16[3];
	uint8_t accuracy_flag;
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_CCGO_CONFIG_SIZE];
		float flt; /* ensure correct memory alignment */
	} C_buffer;
} sCalMag;

static struct {
	int32_t rv_quat_q30[4];
	int32_t rv_accuracy;
	union {
		uint8_t buf[ALGO_INVN_ORIENTATION_CONFIG_SIZE];
		float flt; /* ensure correct memory alignment */
	} C_buffer;
} sRV;

static struct {
	int32_t georv_quat_q30[4];
	int32_t georv_accuracy;
	union {
		uint8_t buf[ALGO_INVN_ORIENTATION_CONFIG_SIZE];
		float flt; /* ensure correct memory alignement */
	} C_buffer;
} sGeoRV;
#endif

/* 
 * Mounting matrix configuration applied for both Accel and Gyro 
 * The coefficient values are coded in integer q30
 */
static int32_t cfg_mounting_matrix[9]= { 1.f*(1<<30), 0,           0,
                                         0,           1.f*(1<<30), 0,
                                         0,           0,           1.f*(1<<30) };

#if USE_AK09911_MAG
/* 
 * Mounting matrix configuration applied for Mag 
 * The coefficient values are coded in integer q30
 */
static int32_t cfg_mag_mounting_matrix[9]= { -1.f*(1<<30), 0,           0,
                                              0,          -1.f*(1<<30), 0,
                                              0,           0,           1.f*(1<<30) };
#endif

/*
 * Dynamic protocol and transport handles
 */
static DynProtocol_t protocol;
static DynProTransportUart_t transport;

/******************************************************************************/

/* Forward declaration */
static int icm20602_sensor_setup(void);
static int icm20602_sensor_configuration(void);
static int icm20602_run_selftest(void);
static void apply_mouting_matrix(const int32_t mounting_matrix[9], const int16_t raw[3], int32_t out[3]);
static void apply_stored_offsets(void);
static void store_offsets(void);
static int sensor_control(int enable);
static int sensor_configure_odr(int odr_us);
static void sensor_event(const inv_sensor_event_t * event, void * arg);
static void algorithms_init(void);
static void algorithms_process(void);
static void notify_event(uint64_t timestamp);
static int handle_command(enum DynProtocolEid eid, const DynProtocolEdata_t * edata, DynProtocolEdata_t * respdata);
static void iddwrapper_protocol_event_cb(enum DynProtocolEtype etype, enum DynProtocolEid eid, const DynProtocolEdata_t * edata, void * cookie);
static void iddwrapper_transport_event_cb(enum DynProTransportEvent e, union DynProTransportEventData data, void * cookie);
static void convert_sensor_event_to_dyn_prot_data(const inv_sensor_event_t * event, VSensorDataAny * vsensor_data);
static void ext_interrupt_main_uart_cb(void * context);
void ext_interrupt_cb(void * context, int int_num);
static int idd_io_hal_init(void);
static int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
#if USE_AK09911_MAG
static uint32_t period_us_to_frequency(const uint32_t period_us);
static int ak09911_sensor_setup(void);
static void algorithms_mag_process(void);
static void notify_mag_event(uint64_t timestamp);
void interrupt_timer_mag_ready_cb(void* context);
void interrupt_timer_start_mag_cb(void* context);
static int idd_io_hal_init_ak09911(void);
static int idd_io_hal_read_reg_ak09911(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int idd_io_hal_write_reg_ak09911(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
#endif
static void check_rc(int rc, const char * context_str);
#ifdef INV_MSG_ENABLE
static void msg_printer(int level, const char * str, va_list ap);
#endif

int main(void)
{
	int rc = 0;
	uart_init_struct_t uart_config;
    
    /*
     * Set NVIC priority according to Nucleo LLD usage
     */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    
	/*
	 * Init the Flash manager by linking to the reserved section in NV memory if any
	 */
	flash_linker_init();

	/*
	 * Init timer peripheral for delay
	 */
	delay_init(DELAY_TIMER);

	/*
	 * Init UART
	 * LOG: USART6 - TX(PA11) / RX(PA12) - 921600 baud ; 8bits ; no parity - TX IRQ enabled
	 * MAIN IddWrapper: USART6 - TX(PA9) / RX(PA10) / CTS(PA11) / RTS(PA12) - 2000000 baud ; 8bits ; no parity - IRQs enabled
	 */
	uart_config.uart_num = LOG_UART_ID;
	uart_config.irqs_on = 1;
	uart_config.use_for_printf = 1;
	uart_config.use_dma_for_tx = 0;
	uart_config.use_dma_for_rx = 0;
	uart_config.tx_buffer = uart_log_buffer;
	uart_config.rx_buffer = NULL;
	uart_config.tx_size = UART_LOG_FIFO_SIZE;
	uart_config.rx_size = 0;
	uart_config.baudrate = 921600;
	uart_config.rx_interrupt_cb = NULL;
	uart_config.tx_interrupt_cb = NULL;
	uart_config.rx_context = NULL;	
	uart_config.tx_context = NULL;
	uart_init(&uart_config);
	
	uart_config.uart_num = MAIN_UART_ID;
	uart_config.irqs_on = 1;
	uart_config.use_for_printf = 0;
	uart_config.use_dma_for_tx = 1;
	uart_config.use_dma_for_rx = 1;
	uart_config.tx_buffer = uart_mainTx_buffer;
	uart_config.rx_buffer = uart_mainRx_buffer;
	uart_config.tx_size = UART_TX_FIFO_SIZE;
	uart_config.rx_size = UART_RX_FIFO_SIZE;
	uart_config.baudrate = 2000000;
	uart_config.rx_interrupt_cb = ext_interrupt_main_uart_cb;
	uart_config.tx_interrupt_cb = NULL;
	uart_config.rx_context = NULL;
	uart_config.tx_context = NULL;
	uart_init(&uart_config);
	/* Prepare DMA to receive 1 byte for any incoming command */
	uart_dma_rx(MAIN_UART_ID);

	/*
	 * Setup message facility to see internal traces from IDD and FW
	 */
	INV_MSG_SETUP(INV_MSG_ENABLE, msg_printer);

	/*
	 * Welcome message
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   ICM20602 LITE example   #");
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################");

	/* 
	* Initialize icm20602 serif structure 
	*/
	struct inv_icm20602_serif icm20602_serif;

	idd_io_hal_init();
	icm20602_serif.context   = 0; /* no need */
	icm20602_serif.read_reg  = idd_io_hal_read_reg;
	icm20602_serif.write_reg = idd_io_hal_write_reg;
#if SERIF_TYPE_SPI
	/*
	 * Init SPI communication: SPI1 - SCK(PA5) / MISO(PA6) / MOSI(PA7) / CS(PB6)
	 */
	icm20602_serif.max_read  = 1024*32; /* maximum number of bytes allowed per serial read */
	icm20602_serif.max_write = 1024*32; /* maximum number of bytes allowed per serial write */
	icm20602_serif.is_spi    = 1;
	INV_MSG(INV_MSG_LEVEL_INFO, "Openning serial interface through SPI");
#elif SERIF_TYPE_I2C
	/* 
	 * Init I2C communication: I2C1 - SCL(PB8) / SDA(PB9)
	 */
	icm20602_serif.max_read  = 64; /* maximum number of bytes allowed per serial read */
	icm20602_serif.max_write = 64; /* maximum number of bytes allowed per serial write */
	icm20602_serif.is_spi    = 0;
	INV_MSG(INV_MSG_LEVEL_INFO, "Openning serial interface through I2C");
#else
	#error "No serial interface selected"
#endif

	/* 
	 * Reset icm20602 driver states
	 */
	inv_icm20602_reset_states(&icm_device, &icm20602_serif);

	/*
	 * Setup the icm20602 device
	 */
	icm20602_sensor_setup();
	icm20602_sensor_configuration();

#if USE_AK09911_MAG
	/* 
	* Initialize ak0991x serif structure 
	*/
	struct inv_ak0991x_serif ak0991x_serif;

	idd_io_hal_init_ak09911();
	ak0991x_serif.context   = 0; /* no need */
	ak0991x_serif.read_reg  = idd_io_hal_read_reg_ak09911;
	ak0991x_serif.write_reg = idd_io_hal_write_reg_ak09911;
	ak0991x_serif.max_read  = 64; /* maximum number of bytes allowed per serial read */
	ak0991x_serif.max_write = 64; /* maximum number of bytes allowed per serial write */
	ak0991x_serif.is_spi    = 0;

	/* 
	 * Reset ak0991x driver states 
	 */
	inv_ak0991x_reset_states(&ak_device, &ak0991x_serif);

	/*
	 * Setup the ak09911 device
	 */
	if (ak09911_sensor_setup()) {
		ak09911_is_available = 0;
		INV_MSG(INV_MSG_LEVEL_WARNING, "Ak09911 setup failed !");
	} else
		ak09911_is_available = 1;
#endif

	/*
	 * Initialize Dynamic protocol stuff
	 */
	DynProTransportUart_init(&transport, iddwrapper_transport_event_cb, 0);
	DynProtocol_init(&protocol, iddwrapper_protocol_event_cb, 0);

	/*
	 * Retrieve offsets stored in NV memory
	 */
	apply_stored_offsets();

	/* 
	 * Initializes the calibration and orientation algorithms
	 */
	algorithms_init();

	/*
	 * Initializes the default sensor ODR in order to properly init the algorithms
	 */
	sensor_configure_odr(period_us);

	/*
	 * Configure the timer input capture mode connected to the external interrupt INT1(D6) dataready to get accurate timestamping
	 * INT1(D6) enabled for dataready interrupt from icm20602
	 */
	timer_configure_timebase(TIMEBASE_TIMER, 1000000);
	timer_configure_irq_capture(TO_MASK(GPIO_SENSOR_IRQ_D6), ext_interrupt_cb, 0);

#if USE_AK09911_MAG
	if ((TIMEBASE_TIMER != MAG_SAMPLING_TIMER) && ak09911_is_available) {
		/* 
		* Configure timer's timebase only if not already in use 
		*/
		timer_configure_timebase(MAG_SAMPLING_TIMER, 100000);
	}
#endif

	/*
	 * At boot time, all sensors are turned on.
	 */
	sensor_control(1);

	do {
		/*
		 * Read RX bytes and feed IddWrapper transport (will ultimatly call IddWrapper event handler)
		 */
		if (irq_event_main_uart) {
			uart_dma_rx_buffer_t * dma_buffer = (uart_dma_rx_buffer_t *)uart_dma_rx_buffer;
			uint8_t idx = 0;
			while(dma_buffer->len--)
				DynProTransportUart_rxProcessByte(&transport, dma_buffer->data[idx++]);
			/* 
			 * Reset buffer length and release current buffer since fully parsed and not needed anymore 
			 */
			dma_buffer->len = 0;
			if (uart_dma_rx_release_buffer(&dma_buffer))
				INV_MSG(INV_MSG_LEVEL_ERROR, "Try to release DMA RX buffer but is empty !");
			__disable_irq();
			irq_event_main_uart = 0;
			__enable_irq();
		}

#if USE_AK09911_MAG
		if (irq_start_mag_capture) {
			/* 
			* Trigger one-shot capture
			*/
			rc = inv_ak0991x_enable_sensor(&ak_device, 1);
			check_rc(rc, "error while starting mag sampling");
			irq_start_mag_capture = 0;
			/* 
			* Prevent from multiple channel allocation 
			*/
			if(mag_end_capture_channel == -1)
				mag_end_capture_channel = timer_configure_callback(MAG_SAMPLING_TIMER, FREQ_TIMER_END_CAPTURE, 0, interrupt_timer_mag_ready_cb);
		}
#endif

		/*
		 * Poll devices for data
		 */
#if USE_AK09911_MAG
		if (irq_from_device & TO_MASK(GPIO_SENSOR_IRQ_D5)) {
			uint64_t timestamp = 0;
			int16_t raw_mag[3];

			timer_channel_stop(MAG_SAMPLING_TIMER, mag_end_capture_channel);
			mag_end_capture_channel = -1;

			rc = inv_ak0991x_poll_data(&ak_device, raw_mag);
			check_rc(rc, "Error %d while polling the ak09911 device");
			__disable_irq();
			if (!RINGBUFFER_EMPTY(&mag_timestamp_buffer))
				RINGBUFFER_POP(&mag_timestamp_buffer, &timestamp);
			__enable_irq();

			/*
			 * Apply the mounting matrix configuration to the mag data polled
			 */
			apply_mouting_matrix(cfg_mag_mounting_matrix, raw_mag, sRmag_data);

			/*
			 * Compute calibration and orientation algorithms
			 */
			algorithms_mag_process();

			/* 
			 * Notify upon new sensor data event
			 */
			notify_mag_event(timestamp);

			__disable_irq();
			irq_from_device &= ~TO_MASK(GPIO_SENSOR_IRQ_D5);
			__enable_irq();
		}
#endif

		if (irq_from_device & TO_MASK(GPIO_SENSOR_IRQ_D6)) {
			uint8_t ddry = 0;
			uint8_t int_status;
			int16_t raw_acc[3], raw_gyro[3];

			/*
			 *  Ensure data ready status
			*/
			if((rc = inv_icm20602_get_int_status(&icm_device, &int_status)) == 0)
				ddry = inv_icm20602_check_drdy(&icm_device, int_status);

			if(ddry) {
				struct inv_icm20602_fifo_states fifo_states;

				rc = inv_icm20602_poll_fifo_data_setup(&icm_device, &fifo_states, int_status);
				check_rc(rc, "Error %d while polling the icm20602 device");
				if(rc == 1) {
					/* 
					 * Overflow detected
					 */
					INV_MSG(INV_MSG_LEVEL_WARNING, "FIFO overflow detected!");
					inv_icm20602_reset_fifo(&icm_device);
					timer_clear_irq_timestamp(TO_MASK(GPIO_SENSOR_IRQ_D6));
				}
				else if(fifo_states.packet_count > 0 && fifo_states.packet_size > 0) {
					/*
					 * Read FIFO only when data is expected in FIFO 
					 */
					while((rc = inv_icm20602_poll_fifo_data(&icm_device, &fifo_states, raw_acc, &sRtemp_data, raw_gyro, NULL)) > 0) {

						uint64_t timestamp = timer_get_irq_timestamp(TO_MASK(GPIO_SENSOR_IRQ_D6));

						/* 
						 * Drop the first timestamp(s) caught by the interrupt 
						 * because the first data in FIFO is always dropped by
						 * the icm20602 driver. 6-axis fusion needs two 
						 * samples to be dropped.
						 */
						while (timestamp_to_drop > 0) {
							timestamp = timer_get_irq_timestamp(TO_MASK(GPIO_SENSOR_IRQ_D6));
							timestamp_to_drop--;
						}

						/*
						* Apply the mounting matrix configuration to the data polled
						*/
						apply_mouting_matrix(cfg_mounting_matrix, raw_acc, sRacc_data);
						apply_mouting_matrix(cfg_mounting_matrix, raw_gyro, sRgyro_data);

						/*
						 * Compute calibration and orientation algorithms
						 */
						algorithms_process();

						/* 
						 * Notify upon new sensor data event
						 */
						notify_event(timestamp);
					}
				}
			}
			__disable_irq();
			irq_from_device &= ~TO_MASK(GPIO_SENSOR_IRQ_D6);
			__enable_irq();
		}
		/* Can do something else or go to sleep until next irq */ 

	} while(1);
}

int icm20602_sensor_setup(void)
{
	int rc;
	uint8_t i, whoami = 0xff;

	/*
	 * Just get the whoami
	 */
	rc = inv_icm20602_get_whoami(&icm_device, &whoami);
	INV_MSG(INV_MSG_LEVEL_INFO, "ICM20602 WHOAMI=0x%02x", whoami);
	check_rc(rc, "Error reading WHOAMI");

	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}

	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x. Expected 0x12, 0x11.", whoami);
		check_rc(-1, "");
	}

	rc = inv_icm20602_get_chip_info(&icm_device, chip_info);
	check_rc(rc, "Could not obtain chip info");

	/*
	 * Configure and initialize the ICM20602 for normal use
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Booting up icm20602...");

	/* set default power mode */
	if (!inv_icm20602_is_sensor_enabled(&icm_device, INV_ICM20602_SENSOR_GYRO) &&
		!inv_icm20602_is_sensor_enabled(&icm_device, INV_ICM20602_SENSOR_ACCEL)) {
		INV_MSG(INV_MSG_LEVEL_VERBOSE, "Putting icm20602 in sleep mode...");
		rc = inv_icm20602_initialize(&icm_device);
		check_rc(rc, "Error %d while setting-up icm20602 device");
	}

	/* set default ODR = 50Hz */
	rc = inv_icm20602_set_sensor_period(&icm_device, INV_ICM20602_SENSOR_ACCEL, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up icm20602 device");

	rc = inv_icm20602_set_sensor_period(&icm_device, INV_ICM20602_SENSOR_GYRO, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up icm20602 device");

	period_us = DEFAULT_ODR_US;

	/* we should be good to go ! */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	return 0;
}

int icm20602_sensor_configuration(void)
{
	int rc;

	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring accelerometer FSR");
	rc = inv_icm20602_set_accel_fullscale(&icm_device, inv_icm20602_accel_fsr_2_reg(cfg_acc_fsr));
	check_rc(rc, "Error configuring ACC sensor");

	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring gyroscope FSR");
	rc = inv_icm20602_set_gyro_fullscale(&icm_device, inv_icm20602_gyro_fsr_2_reg(cfg_gyr_fsr));
	check_rc(rc, "Error configuring GYR sensor");

	return rc;
}

static void apply_mouting_matrix(const int32_t mounting_matrix[9], const int16_t raw[3], int32_t out[3])
{
	unsigned i;

	for(i = 0; i < 3; i++) {
		out[i]  = (int32_t)((int64_t)mounting_matrix[3*i+0]*raw[0] >> 30);
		out[i] += (int32_t)((int64_t)mounting_matrix[3*i+1]*raw[1] >> 30);
		out[i] += (int32_t)((int64_t)mounting_matrix[3*i+2]*raw[2] >> 30);
	}
}

void apply_stored_offsets(void)
{
	uint8_t sensor_bias[84];
	int32_t raw_bias[12] = {0};
	uint8_t i, idx = 0;
	
	/* Retrieve offsets stored in NV memory */
	if(flash_manager_readData(sensor_bias) != 0) {
		INV_MSG(INV_MSG_LEVEL_WARNING, "No bias values retrieved from NV memory !");
		return;
	}
	
	for(i = 0; i < 12; i++)
		raw_bias[i] = inv_dc_little8_to_int32((const uint8_t *)(&sensor_bias[i * sizeof(uint32_t)]));
	idx += sizeof(raw_bias);
	inv_icm20602_set_st_bias(&icm_device, (int *)raw_bias);
	
	for(i = 0; i < 3; i++)
		sCalAcc.acc_bias_q16[i] = inv_dc_little8_to_int32((const uint8_t *)(&sensor_bias[idx + i * sizeof(uint32_t)]));
	idx += sizeof(sCalAcc.acc_bias_q16);
	
	for(i = 0; i < 3; i++)
		sCalGyr.gyr_bias_q16[i] = inv_dc_little8_to_int32((const uint8_t *)(&sensor_bias[idx + i * sizeof(uint32_t)]));
	idx += sizeof(sCalGyr.gyr_bias_q16);
	
#if USE_AK09911_MAG
	for(i = 0; i < 3; i++)
		sCalMag.mag_bias_q16[i] = inv_dc_little8_to_int32((const uint8_t *)(&sensor_bias[idx + i * sizeof(uint32_t)]));
#endif
}

int icm20602_run_selftest(void)
{
	int raw_bias[12];
	int rc = 0;

	if (icm_device.selftest_done == 1) {
		INV_MSG(INV_MSG_LEVEL_INFO, "Self-test has already ran. Skipping.");
	}
	else {
		/* 
		 * Perform self-test
		 * For ICM20602 self-test is performed for both RAW_ACC/RAW_GYR
		 */
		INV_MSG(INV_MSG_LEVEL_INFO, "Running self-test...");

		/* Run the self-test */
		rc = inv_icm20602_run_selftest(&icm_device);
		/* Check transport errors */
		check_rc(rc, "Self-test failure");
		if (rc != 0x3) {
			/*
			 * Check for GYR success (1 << 0) and ACC success (1 << 1),
			 * but don't block as these are 'usage' failures.
			 */
			INV_MSG(INV_MSG_LEVEL_ERROR, "Self-test failure");
			/* 0 would be considered OK, we want KO */
			return INV_ERROR;
		} else
			/* On success, offset will be kept until reset */
			icm_device.selftest_done = 1;

		/* It's advised to re-init the icm20602 device after self-test for normal use */
		rc = icm20602_sensor_setup();
	}

	/* 
	 * Get Low Noise / Low Power bias computed by self-tests scaled by 2^16
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Getting LP/LN bias");
	inv_icm20602_get_st_bias(&icm_device, raw_bias);
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (FS=250dps) (dps): x=%f, y=%f, z=%f", 
			(float)(raw_bias[0] / (float)(1 << 16)), (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LP bias (FS=250dps) (dps): x=%f, y=%f, z=%f", 
			(float)(raw_bias[3] / (float)(1 << 16)), (float)(raw_bias[4] / (float)(1 << 16)), (float)(raw_bias[5] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (FS=2g) (g): x=%f, y=%f, z=%f", 
			(float)(raw_bias[0 + 6] / (float)(1 << 16)), (float)(raw_bias[1 + 6] / (float)(1 << 16)), (float)(raw_bias[2 + 6] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LP bias (FS=2g) (g): x=%f, y=%f, z=%f", 
			(float)(raw_bias[3 + 6] / (float)(1 << 16)), (float)(raw_bias[4 + 6] / (float)(1 << 16)), (float)(raw_bias[5 + 6] / (float)(1 << 16)));

	return rc;
}

void store_offsets(void)
{
	uint8_t i, idx = 0;
	int raw_bias[12] = {0};
	uint8_t sensor_bias[84] = {0};
	
	/* Strore offsets in NV memory */
	inv_icm20602_get_st_bias(&icm_device, raw_bias);
	for(i = 0; i < 12; i++)
		inv_dc_int32_to_little8(raw_bias[i], &sensor_bias[i * sizeof(uint32_t)]);
	idx += sizeof(raw_bias);
	
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(sCalAcc.acc_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
	idx += sizeof(sCalAcc.acc_bias_q16);
	
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(sCalGyr.gyr_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
	idx += sizeof(sCalGyr.gyr_bias_q16);
	
#if USE_AK09911_MAG
	for(i = 0; i < 3; i++)
		inv_dc_int32_to_little8(sCalMag.mag_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
#endif

	flash_manager_writeData(sensor_bias);
}

#if USE_AK09911_MAG
/*
 * Convert a period in usec in a frequency
 */
static uint32_t period_us_to_frequency(const uint32_t period_us)
{
	uint32_t frequency = (1000000 / period_us);
	
	/* Round up frequency */
	if (period_us != 1000000 / frequency)
		frequency++;

	return frequency;
}

int ak09911_sensor_setup(void)
{
	int rc;
	uint8_t whoami;

	INV_MSG(INV_MSG_LEVEL_INFO, "Booting up Ak09911...");

	/* Device soft reset */
	rc = inv_ak0991x_soft_reset(&ak_device);
	if(rc != 0)
		return rc;
 
	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Reading Ak09911 WHOAMI...");
	if((rc = inv_ak0991x_get_whoami(&ak_device, &whoami)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d when reading WHOAMI value", rc);
		return rc;
	}

	if(whoami == 0 || whoami == 0xff) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Unexpected WHOAMI value 0x%x. Aborting setup.", whoami);
		return INV_ERROR;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "AK09911 WHOAMI value: 0x%x", whoami);
	}
	/* register sensitivity adjustment values */
	inv_ak0991x_retrieve_asa_values(&ak_device);

	/* we should be good to go ! */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	/* set default ODR = 50Hz by configuring timer to trigger the magnetometer data acquisition */
	timer_channel_reconfigure_freq(MAG_SAMPLING_TIMER, mag_start_capture_channel, period_us_to_frequency(DEFAULT_ODR_US));

	return 0;
}
#endif

static enum sensor idd_sensortype_conversion(int sensor)
{
	switch(sensor) {
	case INV_SENSOR_TYPE_RAW_ACCELEROMETER:      return SENSOR_RAW_ACC;
	case INV_SENSOR_TYPE_RAW_GYROSCOPE:          return SENSOR_RAW_GYR;
	case INV_SENSOR_TYPE_ACCELEROMETER:          return SENSOR_ACC;
	case INV_SENSOR_TYPE_GYROSCOPE:              return SENSOR_GYR;
	case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:        return SENSOR_UGYR;
	case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:   return SENSOR_GRV;
	case INV_SENSOR_TYPE_PREDICTIVE_QUATERNION:  return hmd_features_supported ? SENSOR_PREDQUAT: SENSOR_MAX;
	case INV_SENSOR_TYPE_GRAVITY:                return SENSOR_GRA;
	case INV_SENSOR_TYPE_LINEAR_ACCELERATION:    return SENSOR_LINACC;
#if USE_AK09911_MAG
	case INV_SENSOR_TYPE_RAW_MAGNETOMETER:       return ak09911_is_available ? SENSOR_RAW_MAG : SENSOR_MAX;
	case INV_SENSOR_TYPE_MAGNETOMETER:           return ak09911_is_available ? SENSOR_MAG     : SENSOR_MAX;
	case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:     return ak09911_is_available ? SENSOR_UMAG    : SENSOR_MAX;
	case INV_SENSOR_TYPE_ROTATION_VECTOR:        return ak09911_is_available ? SENSOR_RV      : SENSOR_MAX;
	case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR: return ak09911_is_available ? SENSOR_GEORV   : SENSOR_MAX;
#endif
	default:                                     return SENSOR_MAX;
	}
}

int sensor_control(int enable)
{
	int rc = 0;
	static uint8_t sensors_on = 0;

	/* Keep track of the sensors state */
	if(enable && sensors_on)
		return rc;

	if(enable)
		sensors_on = 1;
	else 
		sensors_on = 0;

	/* Handling of Game Rotation Vector (6-axis AG) */
	if (enable) {
		/* Handles the orientation algoritm state */
		Algo_InvnOrientation_BodyToWorldFrameFxp_ResetStates(sGRV.C_buffer.buf);
		Algo_InvnOrientation_BodyToWorldFrameFxp_AG_Enable(sGRV.C_buffer.buf);
	} else
		Algo_InvnOrientation_BodyToWorldFrameFxp_AG_Disable(sGRV.C_buffer.buf);

	/* Handling of Predictive Quaternion (6-axis AG) */
	if (enable && hmd_features_supported) {
		/* Handles the orientation algoritm state */
		Algo_InvnOrientation_PredictiveQuaternionFxp_ResetStates(sPredGRV.C_buffer.buf);
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetSamplingPeriod(sPredGRV.C_buffer.buf, period_us);
	}

#if USE_AK09911_MAG
	if (ak09911_is_available) {
		/* Handling of Rotation Vector (9-axis AGM) */
		if (enable) {
			/* Handles the orientation algoritm state */
			Algo_InvnOrientation_BodyToWorldFrameFxp_ResetStates(sRV.C_buffer.buf);
			Algo_InvnOrientation_BodyToWorldFrameFxp_AGM_Enable(sRV.C_buffer.buf);
		} else
			Algo_InvnOrientation_BodyToWorldFrameFxp_AGM_Disable(sRV.C_buffer.buf);

		/* Handling of Geomagnetic Rotation Vector (6-axis AGM) */
		if (enable) {
			/* Handles the orientation algoritm state */
			Algo_InvnOrientation_BodyToWorldFrameFxp_ResetStates(sGeoRV.C_buffer.buf);
			Algo_InvnOrientation_BodyToWorldFrameFxp_AM_Enable(sGeoRV.C_buffer.buf);
		} else
			Algo_InvnOrientation_BodyToWorldFrameFxp_AM_Disable(sGeoRV.C_buffer.buf);
	}
#endif

	/*
	 *  Call driver APIs to start/stop sensors
	 */
	if (enable) {
		/* Clock is more accurate when gyro is enabled, so let's enable it first to prevent side effect at startup */
		if (!inv_icm20602_is_sensor_enabled(&icm_device, INV_ICM20602_SENSOR_GYRO))
			rc += inv_icm20602_enable_sensor(&icm_device, INV_ICM20602_SENSOR_GYRO, 1);
		if (!inv_icm20602_is_sensor_enabled(&icm_device, INV_ICM20602_SENSOR_ACCEL))
			rc += inv_icm20602_enable_sensor(&icm_device, INV_ICM20602_SENSOR_ACCEL, 1);
		if (!inv_icm20602_is_sensor_enabled(&icm_device, INV_ICM20602_SENSOR_TEMPERATURE))
			rc += inv_icm20602_enable_sensor(&icm_device, INV_ICM20602_SENSOR_TEMPERATURE, 1);
		/*
		 * There is a situation where two samples need to be dropped: if
		 * accelerometer is enable before gyroscope first interrupt triggers,
		 * both interrupts are raised causing the odr to be wrong if only one
		 * sample is dropped.
		 * We are in this exact situation since both sensors are enabled one after
		 * the other.
		 */
		timestamp_to_drop = 2;
	} else {
		rc += inv_icm20602_enable_sensor(&icm_device, INV_ICM20602_SENSOR_GYRO, 0);
		rc += inv_icm20602_enable_sensor(&icm_device, INV_ICM20602_SENSOR_ACCEL, 0);
		rc += inv_icm20602_enable_sensor(&icm_device, INV_ICM20602_SENSOR_TEMPERATURE, 0);
	}


#if USE_AK09911_MAG
	if (ak09911_is_available) {
		if (enable && mag_start_capture_channel == -1) {
			/* Configure the timer to trigger the magnetometer data capture */
			uint32_t mag_period_us = period_us;
			if(mag_period_us < MIN_MAG_ODR_US)
				mag_period_us = MIN_MAG_ODR_US;
			mag_start_capture_channel = timer_configure_callback(MAG_SAMPLING_TIMER, period_us_to_frequency(mag_period_us), 0, interrupt_timer_start_mag_cb);
		} else if (!enable && mag_start_capture_channel != -1) {
			/* Stop the timers used for the magnetometer data capture */
			/* Do not check return code since it may introduce false positive (main loop clears it) */
			timer_channel_stop(MAG_SAMPLING_TIMER, mag_end_capture_channel);
			mag_end_capture_channel = -1;
			rc = timer_channel_stop(MAG_SAMPLING_TIMER, mag_start_capture_channel);
			mag_start_capture_channel = -1;
			irq_start_mag_capture = 0;

			__disable_irq();
			RINGBUFFER_CLEAR(&mag_timestamp_buffer);
			__enable_irq();
			rc += inv_ak0991x_enable_sensor(&ak_device, 0);
		}
	}
#endif

	/* Clear the remaining items in the IRQ timestamp buffer when stopping all sensors */
	if(inv_icm20602_all_sensors_off(&icm_device))
		rc += timer_clear_irq_timestamp(TO_MASK(GPIO_SENSOR_IRQ_D6));

	return rc;
}

int sensor_configure_odr(int odr_us)
{
	int rc = 0;
#if USE_AK09911_MAG
	int mag_odr_us;
#endif

	/* All sensors running at the same rate */

	/* Do not reconfigure the rate if it's already applied */
	if(odr_us == period_us)
		return rc;

	/*
	 * Maximum supported rate is 1kHz
	 */
	if(odr_us < MIN_ODR_US)
		odr_us = MIN_ODR_US;

	/* 
	 * Minimum rate supported is 50Hz 
	 * - To compute a correct orientation with the GRV algorithm
	 * - For accelerometer sensor using gyro-assisted calibration.
	 * The basic accelerometer and gyroscope sensor could report at lower frequency, but we simplify the example with this condition.
	 */
	if(odr_us > MAX_ODR_US)
		odr_us = MAX_ODR_US;

#if USE_AK09911_MAG
	/* 
	 * The maximum rate available for the magnetometer is 100Hz
	 */
	mag_odr_us = odr_us;
	if(mag_odr_us < MIN_MAG_ODR_US)
		mag_odr_us = MIN_MAG_ODR_US;
#endif

	/*
	 *  Call driver APIs to start/stop sensors
	 */
	rc = inv_icm20602_set_sensor_period(&icm_device, INV_ICM20602_SENSOR_ACCEL, odr_us / 1000);
	rc += inv_icm20602_set_sensor_period(&icm_device, INV_ICM20602_SENSOR_GYRO, odr_us / 1000);
	/* FIFO has been reset by ODR change */
	if (rc == 0) {
		rc += timer_clear_irq_timestamp(TO_MASK(GPIO_SENSOR_IRQ_D6));
		/* Clear any remaining interrupts */
		__disable_irq();
		irq_from_device &= ~TO_MASK(GPIO_SENSOR_IRQ_D6);
		__enable_irq();
	}
#if USE_AK09911_MAG
	if (ak09911_is_available) {
		timer_channel_reconfigure_freq(MAG_SAMPLING_TIMER, mag_start_capture_channel, period_us_to_frequency(mag_odr_us));
		__disable_irq();
		/* Clear timestamp buffer to remove remaining interrupt catches */
		RINGBUFFER_CLEAR(&mag_timestamp_buffer);
		/* Clear any remaining interrupts */
		irq_from_device &= ~TO_MASK(GPIO_SENSOR_IRQ_D5);
		__enable_irq();
	}
#endif

	/* Keep track in static variable of the odr value for further algorihtm use */
	period_us = odr_us;

	/* 
	 * Update algorithm parameters for Gyroscope calibration
	 */
	Algo_InvnCalibration_GyroCalibrationFxp_SetSamplingPeriod(sCalGyr.C_buffer.buf, odr_us);

	/* 
	 * Update algorithm parameters for Accelerometer calibration
	 */
	Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_SetSamplingPeriod(sCalAcc.C_buffer.buf, odr_us);

#if USE_AK09911_MAG
	/* 
	 * Update algorithm parameters for Magnetometer calibration
	 */
	if (ak09911_is_available)
		Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_SetSamplingPeriod(sCalMag.C_buffer.buf, mag_odr_us);
#endif

	/* 
	 * Update algorithm parameters for GRV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_SetGyrSamplingPeriod(sGRV.C_buffer.buf, odr_us, chip_info);
	Algo_InvnOrientation_BodyToWorldFrameFxp_SetAccSamplingPeriod(sGRV.C_buffer.buf, odr_us, chip_info);
	
	/* 
	 * Update algorithm parameters for Predictive Quaternion orientation
	 */
	if(hmd_features_supported)
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetSamplingPeriod(sPredGRV.C_buffer.buf, odr_us);

#if USE_AK09911_MAG
	/* 
	 * Update algorithm parameters for RV orientation
	 */
	if (ak09911_is_available) {
		Algo_InvnOrientation_BodyToWorldFrameFxp_SetGyrSamplingPeriod(sRV.C_buffer.buf, odr_us, chip_info);
		Algo_InvnOrientation_BodyToWorldFrameFxp_SetAccSamplingPeriod(sRV.C_buffer.buf, odr_us, chip_info);
		Algo_InvnOrientation_BodyToWorldFrameFxp_SetMagSamplingPeriod(sRV.C_buffer.buf, odr_us);
	}
#endif

	return rc;
}

void sensor_event(const inv_sensor_event_t * event, void * arg)
{
	/* arg will contained the value provided at init time */
	(void)arg;

	/*
	 * Encode sensor event and sent to host over UART through IddWrapper protocol
	 */
	static DynProtocolEdata_t async_edata; /* static to take on .bss */
	uart_dma_tx_buffer_t * dma_buffer;
	DynProTransportUartFrame_t uart_frame;
	int timeout = 5000; /* us */;

	async_edata.sensor_id = event->sensor;
	async_edata.d.async.sensorEvent.status = DYN_PRO_SENSOR_STATUS_DATA_UPDATED;
	convert_sensor_event_to_dyn_prot_data(event, &async_edata.d.async.sensorEvent.vdata);

	while((uart_dma_tx_take_buffer(&dma_buffer) != 0) && (timeout > 0)) {
		inv_icm20602_sleep_us(10);
		timeout -= 10;
	}

	if (timeout <= 0) {
		/* if no available buffer, can't send the sensor data */
		INV_MSG(INV_MSG_LEVEL_WARNING, "sensor_event_cb: 'uart_dma_tx_take_buffer' timeouted, frame dropped");
		return;
	}

	/* get memory location for the iddwrapper protocol payload */
	if(DynProTransportUart_txAssignBuffer(&transport, &uart_frame,
			dma_buffer->data, sizeof(dma_buffer->data)) != 0) {
		goto error_dma_buf;
	}

	if(DynProtocol_encodeAsync(&protocol,
			DYN_PROTOCOL_EID_NEW_SENSOR_DATA, &async_edata,
			uart_frame.payload_data, uart_frame.max_payload_len, &uart_frame.payload_len) != 0) {
		goto error_dma_buf;
	}
	if(DynProTransportUart_txEncodeFrame(&transport, &uart_frame) != 0) {
		goto error_dma_buf;
	}

	/* send async event */
	dma_buffer->len = uart_frame.len;
	if(uart_dma_tx(MAIN_UART_ID, dma_buffer) != 0) {
		INV_MSG(INV_MSG_LEVEL_DEBUG, "sensor_event_cb: 'uart_dma_tx()' returned error");
	}
	return;

error_dma_buf:
	INV_MSG(INV_MSG_LEVEL_WARNING, "sensor_event_cb: encode error, frame dropped");
	uart_dma_tx_release_buffer(&dma_buffer);
	return;
}

void algorithms_init(void)
{
	int32_t gyro_offset_2000dps_q30[3] = {
		((sCalGyr.gyr_bias_q16[0] << 3) / 2000) << (30 - 16 - 3),
		((sCalGyr.gyr_bias_q16[1] << 3) / 2000) << (30 - 16 - 3),
		((sCalGyr.gyr_bias_q16[2] << 3) / 2000) << (30 - 16 - 3)
	};
#if USE_AK09911_MAG
	uint32_t mag_period_us;
#endif

	/* Reset the algorithm and the accuracy to 0 and re-apply known offsets (could be stored in flash) */
	Algo_InvnCalibration_GyroCalibrationFxp_Init(sCalGyr.C_buffer.buf, gyro_offset_2000dps_q30, 0);
	Algo_InvnCalibration_GyroCalibrationFxp_SetUserParam(sCalGyr.C_buffer.buf, 0, HMD_VR_MODE);

	/*
	 * Init the accelerometer calibration
	 */
	Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_Init(sCalAcc.C_buffer.buf, sCalAcc.acc_bias_q16, 0, period_us);

	/*
	 * Init the GRV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_Init(sGRV.C_buffer.buf);
	
	/*
	 * Init the Predictive quaternion orientation
	 */
	if(hmd_features_supported) {
		Algo_InvnOrientation_PredictiveQuaternionFxp_Init(sPredGRV.C_buffer.buf);
		Algo_InvnOrientation_PredictiveQuaternionFxp_SetParam(sPredGRV.C_buffer.buf,
			ALGO_INVN_PREDICTIVEQUATERNION_DEFAULT_METHOD_ENNUM, ALGO_INVN_PREDICTIVEQUATERNION_DEFAULT_PREDICTIVE_TIME_US
		);
	}

#if USE_AK09911_MAG
	/*
	 * Init the compass calibration algorithm
	 */
	mag_period_us = period_us;
	if(mag_period_us < MIN_MAG_ODR_US)
		mag_period_us = MIN_MAG_ODR_US;

	Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_Init(sCalMag.C_buffer.buf, sCalMag.mag_bias_q16, 0, mag_period_us);

	/*
	 * Init the RV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_Init(sRV.C_buffer.buf);
	Algo_InvnOrientation_BodyToWorldFrameFxp_SetMagCustomParams(sRV.C_buffer.buf,
			RV_ANOMALIES_REJECTION,
			RV_THRESHOLD_YAW_STOP_CONVERGENCE,
			RV_THRESHOLD_GYR_STOP_CONVERGENCE,
			RV_THRESHOLD_YAW_SMOOTH_CONVERGENCE_DEFAULT
	);

	/*
	 * Init the GeoRV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_Init(sGeoRV.C_buffer.buf);
#endif

	/* 
	 * Get the HMD features support availability for the chip 
	 */
	hmd_features_supported = Algo_InvnCalibration_isHMDDevice(chip_info);
}

void algorithms_process(void)
{
	/* Get the temperature value by applying the sensitivity 326.8 LSB/degC and adding 25degC offset */
	const int32_t temp_degC_q16 = (((int32_t)(sRtemp_data << 16)) / 3268 * 10) + (int32_t)(25 << 16);
	const int32_t temp_100degC = (temp_degC_q16 * 100) >> 16;

	/*
	 * Compute the calibrated accelerometer data
	 */
	{
		const int32_t raw_accel_q25[3] = {
			(sRacc_data[0] * ACC_SENSITIVITY) << (25 - 16),
			(sRacc_data[1] * ACC_SENSITIVITY) << (25 - 16),
			(sRacc_data[2] * ACC_SENSITIVITY) << (25 - 16),
		};
		int32_t accel_bias_q25[3];
		int32_t accel_cal_q25[3];

		Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_UpdateAcc(sCalAcc.C_buffer.buf, raw_accel_q25, temp_100degC, accel_bias_q25);
		accel_cal_q25[0] = raw_accel_q25[0] - accel_bias_q25[0];
		accel_cal_q25[1] = raw_accel_q25[1] - accel_bias_q25[1];
		accel_cal_q25[2] = raw_accel_q25[2] - accel_bias_q25[2];

		sCalAcc.acc_bias_q16[0] = accel_bias_q25[0] >> (25 - 16);
		sCalAcc.acc_bias_q16[1] = accel_bias_q25[1] >> (25 - 16);
		sCalAcc.acc_bias_q16[2] = accel_bias_q25[2] >> (25 - 16);
		sCalAcc.acc_cal_q16[0] = accel_cal_q25[0] >> (25 -16);
		sCalAcc.acc_cal_q16[1] = accel_cal_q25[1] >> (25 -16);
		sCalAcc.acc_cal_q16[2] = accel_cal_q25[2] >> (25 -16);
		sCalAcc.accuracy_flag = Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_GetAccuracy(sCalAcc.C_buffer.buf);

		/* Update GRV algorithm with acc value and acc accuracy */
		Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateAcc(sGRV.C_buffer.buf, 
				accel_cal_q25, sCalAcc.accuracy_flag);
#if USE_AK09911_MAG
		if (ak09911_is_available) {
			/* Update RV algorithm with acc value and acc accuracy */
			Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateAcc(sRV.C_buffer.buf, 
					accel_cal_q25, sCalAcc.accuracy_flag);
			/* Update GeoRV with acc values and acc accuracy */
			Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateAcc(sGeoRV.C_buffer.buf,
					accel_cal_q25, sCalAcc.accuracy_flag);
		}
#endif
	}

	/*
	 * Compute the calibrated gyroscope data
	 */
	{
		const int32_t raw_gyr_2000dps_q15[3] = {
			(sRgyro_data[0] * GYR_SENSITIVITY) / (2*2000),
			(sRgyro_data[1] * GYR_SENSITIVITY) / (2*2000),
			(sRgyro_data[2] * GYR_SENSITIVITY) / (2*2000)
		};
		int32_t gyro_uncal_2000dps_q30[3];
		int32_t gyro_offset_2000dps_q30[3];

		Algo_InvnCalibration_GyroCalibrationFxp_UpdateGyr(sCalGyr.C_buffer.buf, raw_gyr_2000dps_q15, temp_100degC);
		Algo_InvnCalibration_GyroCalibrationFxp_GetUncalibrated(sCalGyr.C_buffer.buf, gyro_uncal_2000dps_q30);
		Algo_InvnCalibration_GyroCalibrationFxp_GetBias(sCalGyr.C_buffer.buf, gyro_offset_2000dps_q30);
		Algo_InvnCalibration_GyroCalibrationFxp_GetCalibrated(sCalGyr.C_buffer.buf, sCalGyr.gyro_cal_2000dps_q30);
			
		sCalGyr.gyr_uncal_q16[0] = ((gyro_uncal_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_uncal_q16[1] = ((gyro_uncal_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_uncal_q16[2] = ((gyro_uncal_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_bias_q16[0] = ((gyro_offset_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_bias_q16[1] = ((gyro_offset_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_bias_q16[2] = ((gyro_offset_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_cal_q16[0] = ((sCalGyr.gyro_cal_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_cal_q16[1] = ((sCalGyr.gyro_cal_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_cal_q16[2] = ((sCalGyr.gyro_cal_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.accuracy_flag = Algo_InvnCalibration_GyroCalibrationFxp_GetAccuracy(sCalGyr.C_buffer.buf);

		if(hmd_features_supported && (sCalGyr.accuracy_flag == 3)) {
			int32_t cal_gyr_q16_rps[3] = {
				(int32_t) (((int64_t) sCalGyr.gyr_cal_q16[0] * FACTOR_DPS_TO_RPS) >> 30),
				(int32_t) (((int64_t) sCalGyr.gyr_cal_q16[1] * FACTOR_DPS_TO_RPS) >> 30),
				(int32_t) (((int64_t) sCalGyr.gyr_cal_q16[2] * FACTOR_DPS_TO_RPS) >> 30)
			};
			/* Feed gyro-assisted calibration for Accelerometer */
			Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_UpdateGyr(sCalAcc.C_buffer.buf, 
					cal_gyr_q16_rps, period_us, sCalGyr.accuracy_flag, chip_info);
#if USE_AK09911_MAG
			/* Feed gyro-assisted calibration for Magnetometer */
			if (ak09911_is_available)
				Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_UpdateGyr(sCalMag.C_buffer.buf,
						cal_gyr_q16_rps, period_us, sCalGyr.accuracy_flag, chip_info);
#endif
		}

		/* Update GRV with gyr value and gyr accuracy */
		Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateGyr(sGRV.C_buffer.buf, 
				sCalGyr.gyro_cal_2000dps_q30, sCalGyr.accuracy_flag);
#if USE_AK09911_MAG
		/* Update RV with gyr value and gyr accuracy */
		if (ak09911_is_available)
			Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateGyr(sRV.C_buffer.buf, 
					sCalGyr.gyro_cal_2000dps_q30, sCalGyr.accuracy_flag);
#endif
	}

	/*
	 * Compute the game rotation vector data
	 * Note : the orientation may drift until the GRV accuracy flag reaches 3. Once calibrated, the position is kept as initial reference.
	 */
	{
		Algo_InvnOrientation_BodyToWorldFrameFxp_GetGameRotationVector(sGRV.C_buffer.buf, sGRV.grv_quat_q30);
	}


	/*
	 * Compute the predictive quaternion data
	 */
	if(hmd_features_supported) {
		Algo_InvnOrientation_PredictiveQuaternionFxp_Update(sPredGRV.C_buffer.buf, 
				sCalGyr.gyro_cal_2000dps_q30, sGRV.grv_quat_q30, sPredGRV.predgrv_quat_q30, chip_info);
	}

	/*
	 * Compute the gravity data
	 */
	{
		/* x axis */
		sGRV.gravity_q16[0] = (2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[1] * sGRV.grv_quat_q30[3]) >> 30)
				- 2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[0] * sGRV.grv_quat_q30[2]) >> 30)) >> (30 - 16);
		/* y axis */
		sGRV.gravity_q16[1] = (2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[2] * sGRV.grv_quat_q30[3]) >> 30)
				+ 2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[0] * sGRV.grv_quat_q30[1]) >> 30)) >> (30 - 16);
		/* z axis */
		sGRV.gravity_q16[2] = ((1 << 30) - 2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[1] * sGRV.grv_quat_q30[1]) >> 30)
				- 2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[2] * sGRV.grv_quat_q30[2]) >> 30)) >> (30 - 16);
	}

	/*
	 * Compute the linear acceleration data
	 */
	{
		sGRV.linearacc_q16[0] = sCalAcc.acc_cal_q16[0] - sGRV.gravity_q16[0];
		sGRV.linearacc_q16[1] = sCalAcc.acc_cal_q16[1] - sGRV.gravity_q16[1];
		sGRV.linearacc_q16[2] = sCalAcc.acc_cal_q16[2] - sGRV.gravity_q16[2];
	}

#if USE_AK09911_MAG
	/*
	 * Compute the rotation vector data
	 */
	if (ak09911_is_available)
	{
		int32_t heading_accuracy_q27 = Algo_InvnOrientation_BodyToWorldFrameFxp_GetRotationVector(sRV.C_buffer.buf, sRV.rv_quat_q30);
		sRV.rv_accuracy = (int32_t)(((int64_t)heading_accuracy_q27 * (3754936 /* 180/pi * 2^16) */)) >> 27);
	}
#endif
}

#if USE_AK09911_MAG
void algorithms_mag_process(void)
{
	/*
	 * Compute the calibrated magnetometer data
	 */
	{
		const int32_t soft_iron[9] = { 1, 0, 0,
                                       0, 1, 0,
                                       0, 0, 1 };
		int32_t raw_mag_ut_q16[3];
		int32_t local_field_norm_q16;
		unsigned i;

		for(i = 0; i < 3; i++) {
			raw_mag_ut_q16[i]  = (int32_t)((int64_t)soft_iron[3*i+0]*sRmag_data[0]);
			raw_mag_ut_q16[i] += (int32_t)((int64_t)soft_iron[3*i+1]*sRmag_data[1]);
			raw_mag_ut_q16[i] += (int32_t)((int64_t)soft_iron[3*i+2]*sRmag_data[2]);
			raw_mag_ut_q16[i] *= MAG_SENSITIVITY;
		}

		Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_UpdateMag(sCalMag.C_buffer.buf, raw_mag_ut_q16, sCalMag.mag_bias_q16);

		sCalMag.mag_uncal_q16[0] = raw_mag_ut_q16[0];
		sCalMag.mag_uncal_q16[1] = raw_mag_ut_q16[1];
		sCalMag.mag_uncal_q16[2] = raw_mag_ut_q16[2];
		sCalMag.mag_cal_q16[0] = sCalMag.mag_uncal_q16[0] - sCalMag.mag_bias_q16[0];
		sCalMag.mag_cal_q16[1] = sCalMag.mag_uncal_q16[1] - sCalMag.mag_bias_q16[1];
		sCalMag.mag_cal_q16[2] = sCalMag.mag_uncal_q16[2] - sCalMag.mag_bias_q16[2];
		sCalMag.accuracy_flag = Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_GetAccuracy(sCalMag.C_buffer.buf);
		local_field_norm_q16 = Algo_InvnCalibration_CompassCalibrationGyroOptionalFxp_GetFieldNorm(sCalMag.C_buffer.buf);

		/* Update RV with mag values and mag accuracy */
		Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateMag(sRV.C_buffer.buf,
				sCalMag.mag_cal_q16, sCalMag.accuracy_flag, 0 /*the local magnetic field norm is not used for RV*/);
		/* Update GeoRV with gyr values and gyr accuracy */
		Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateMag(sGeoRV.C_buffer.buf, 
				sCalMag.mag_cal_q16, sCalMag.accuracy_flag, local_field_norm_q16);
	}

	/*
	 * Compute the geomagnetic rotation vector data
	 */
	{
		int32_t heading_accuracy_q27 = Algo_InvnOrientation_BodyToWorldFrameFxp_GetGeoMagRotationVector(sGeoRV.C_buffer.buf, sGeoRV.georv_quat_q30);
		sGeoRV.georv_accuracy = (int32_t)(((int64_t)heading_accuracy_q27 * (3754936 /* 180/pi * 2^16) */)) >> 27);
	}
}
#endif

void notify_event(uint64_t timestamp)
{
	inv_sensor_event_t event;
	memset(&event, 0, sizeof(event));

	/* 
	 * New raw accel data 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_RAW_ACC)) {
		event.sensor	= INV_SENSOR_TYPE_RAW_ACCELEROMETER;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.raw3d.vect[0] = sRacc_data[0];
		event.data.raw3d.vect[1] = sRacc_data[1];
		event.data.raw3d.vect[2] = sRacc_data[2];

		sensor_event(&event, NULL);
	}

	/* 
	 * New calibrated accel event
	 */
	if(enabled_sensor_mask & (1 << SENSOR_ACC)) {
		event.sensor	= INV_SENSOR_TYPE_ACCELEROMETER;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.acc.bias[0] = (float) sCalAcc.acc_bias_q16[0] / (1 << 16);
		event.data.acc.bias[1] = (float) sCalAcc.acc_bias_q16[1] / (1 << 16);
		event.data.acc.bias[2] = (float) sCalAcc.acc_bias_q16[2] / (1 << 16);
		event.data.acc.vect[0] = (float) sCalAcc.acc_cal_q16[0] / (1 << 16);
		event.data.acc.vect[1] = (float) sCalAcc.acc_cal_q16[1] / (1 << 16);
		event.data.acc.vect[2] = (float) sCalAcc.acc_cal_q16[2] / (1 << 16);
		event.data.acc.accuracy_flag = sCalAcc.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New raw gyro data 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_RAW_GYR)) {
		event.sensor	= INV_SENSOR_TYPE_RAW_GYROSCOPE;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.raw3d.vect[0] = sRgyro_data[0];
		event.data.raw3d.vect[1] = sRgyro_data[1];
		event.data.raw3d.vect[2] = sRgyro_data[2];

		sensor_event(&event, NULL);
	}

	/* 
	 * New uncalibrated gyro event
	 */
	if(enabled_sensor_mask & (1 << SENSOR_UGYR)) {
		event.sensor	= INV_SENSOR_TYPE_UNCAL_GYROSCOPE;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.gyr.bias[0] = (float) sCalGyr.gyr_bias_q16[0] / (1 << 16);
		event.data.gyr.bias[1] = (float) sCalGyr.gyr_bias_q16[1] / (1 << 16);
		event.data.gyr.bias[2] = (float) sCalGyr.gyr_bias_q16[2] / (1 << 16);
		event.data.gyr.vect[0] = (float) sCalGyr.gyr_uncal_q16[0] / (1 << 16);
		event.data.gyr.vect[1] = (float) sCalGyr.gyr_uncal_q16[1] / (1 << 16);
		event.data.gyr.vect[2] = (float) sCalGyr.gyr_uncal_q16[2] / (1 << 16);
		event.data.gyr.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New calibrated gyro event
	 */
	if(enabled_sensor_mask & (1 << SENSOR_GYR)) {
		event.sensor	= INV_SENSOR_TYPE_GYROSCOPE;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.gyr.bias[0] = (float) sCalGyr.gyr_bias_q16[0] / (1 << 16);
		event.data.gyr.bias[1] = (float) sCalGyr.gyr_bias_q16[1] / (1 << 16);
		event.data.gyr.bias[2] = (float) sCalGyr.gyr_bias_q16[2] / (1 << 16);
		event.data.gyr.vect[0] = (float) sCalGyr.gyr_cal_q16[0] / (1 << 16);
		event.data.gyr.vect[1] = (float) sCalGyr.gyr_cal_q16[1] / (1 << 16);
		event.data.gyr.vect[2] = (float) sCalGyr.gyr_cal_q16[2] / (1 << 16);
		event.data.gyr.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New GRV event 
	 * scheduled on gyroscope data update
	 */
	if(enabled_sensor_mask & (1 << SENSOR_GRV)) {
		event.sensor	= INV_SENSOR_TYPE_GAME_ROTATION_VECTOR;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.quaternion.quat[0] = (float)sGRV.grv_quat_q30[0] / (1 << 30);
		event.data.quaternion.quat[1] = (float)sGRV.grv_quat_q30[1] / (1 << 30);
		event.data.quaternion.quat[2] = (float)sGRV.grv_quat_q30[2] / (1 << 30);
		event.data.quaternion.quat[3] = (float)sGRV.grv_quat_q30[3] / (1 << 30);
		/* Report additionnal accuracy flag, being currently a copy of GYR accuracy flag (ACC flag could also be considered) */
		event.data.quaternion.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}
	
	/* 
	 * New Predictive quaternion event 
	 * scheduled on gyroscope data update
	 */
	if((enabled_sensor_mask & (1 << SENSOR_PREDQUAT)) && hmd_features_supported) {
		event.sensor	= INV_SENSOR_TYPE_PREDICTIVE_QUATERNION;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.quaternion.quat[0] = (float)sPredGRV.predgrv_quat_q30[0] / (1 << 30);
		event.data.quaternion.quat[1] = (float)sPredGRV.predgrv_quat_q30[1] / (1 << 30);
		event.data.quaternion.quat[2] = (float)sPredGRV.predgrv_quat_q30[2] / (1 << 30);
		event.data.quaternion.quat[3] = (float)sPredGRV.predgrv_quat_q30[3] / (1 << 30);
		/* Report additionnal accuracy flag, being currently a copy of GYR accuracy flag (ACC flag could also be considered) */
		event.data.quaternion.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New gravity event 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_GRA)) {
		event.sensor	= INV_SENSOR_TYPE_GRAVITY;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.acc.vect[0] = (float)sGRV.gravity_q16[0] / (1 << 16);
		event.data.acc.vect[1] = (float)sGRV.gravity_q16[1] / (1 << 16);
		event.data.acc.vect[2] = (float)sGRV.gravity_q16[2] / (1 << 16);
		/* Report additionnal accuracy flag, being currently the copy of GRV accuracy flag (copied from GYR accuracy flag) */
		event.data.acc.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New linear acceleration event 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_LINACC)) {
		event.sensor	= INV_SENSOR_TYPE_LINEAR_ACCELERATION;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.acc.vect[0] = (float)sGRV.linearacc_q16[0] / (1 << 16);
		event.data.acc.vect[1] = (float)sGRV.linearacc_q16[1] / (1 << 16);
		event.data.acc.vect[2] = (float)sGRV.linearacc_q16[2] / (1 << 16);
		/* Report additionnal accuracy flag, being currently the copy of GRV accuracy flag (copied from GYR accuracy flag) */
		event.data.acc.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

#if USE_AK09911_MAG
	/* 
	 * New RV event 
	 * scheduled on gyroscope data update
	 */
	if((enabled_sensor_mask & (1 << SENSOR_RV)) && ak09911_is_available) {
		event.sensor	= INV_SENSOR_TYPE_ROTATION_VECTOR;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.quaternion.quat[0]  = (float)sRV.rv_quat_q30[0] / (1 << 30);
		event.data.quaternion.quat[1]  = (float)sRV.rv_quat_q30[1] / (1 << 30);
		event.data.quaternion.quat[2]  = (float)sRV.rv_quat_q30[2] / (1 << 30);
		event.data.quaternion.quat[3]  = (float)sRV.rv_quat_q30[3] / (1 << 30);
		event.data.quaternion.accuracy = (float)sRV.rv_accuracy / (1 << 16);
		/* Report additionnal accuracy flag, being currently  minimum between MAG and GYR (ACC flag could also be considered) */
		if(sCalGyr.accuracy_flag < sCalMag.accuracy_flag)
			event.data.quaternion.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;
		else
			event.data.quaternion.accuracy_flag = sCalMag.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}
#endif
}

#if USE_AK09911_MAG
void notify_mag_event(uint64_t timestamp)
{
	inv_sensor_event_t event;
	memset(&event, 0, sizeof(event));

	/*
	 * New raw mag data
	 */
	if(enabled_sensor_mask & (1 << SENSOR_RAW_MAG)) {
		event.sensor	= INV_SENSOR_TYPE_RAW_MAGNETOMETER;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.raw3d.vect[0] = sRmag_data[0];
		event.data.raw3d.vect[1] = sRmag_data[1];
		event.data.raw3d.vect[2] = sRmag_data[2];

		sensor_event(&event, NULL);
	}

	/* 
	 * New uncalibrated mag event
	*/
	if(enabled_sensor_mask & (1 << SENSOR_UMAG)) {
		event.sensor	= INV_SENSOR_TYPE_UNCAL_MAGNETOMETER;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.mag.bias[0] = (float)sCalMag.mag_bias_q16[0] / (1 << 16);
		event.data.mag.bias[1] = (float)sCalMag.mag_bias_q16[1] / (1 << 16);
		event.data.mag.bias[2] = (float)sCalMag.mag_bias_q16[2] / (1 << 16);
		event.data.mag.vect[0] = (float)sCalMag.mag_uncal_q16[0] / (1 << 16);
		event.data.mag.vect[1] = (float)sCalMag.mag_uncal_q16[1] / (1 << 16);
		event.data.mag.vect[2] = (float)sCalMag.mag_uncal_q16[2] / (1 << 16);
		event.data.mag.accuracy_flag = sCalMag.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New calibrated mag event
	 */
	if(enabled_sensor_mask & (1 << SENSOR_MAG)) {
		event.sensor	= INV_SENSOR_TYPE_MAGNETOMETER;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.mag.bias[0] = (float)sCalMag.mag_bias_q16[0] / (1 << 16);
		event.data.mag.bias[1] = (float)sCalMag.mag_bias_q16[1] / (1 << 16);
		event.data.mag.bias[2] = (float)sCalMag.mag_bias_q16[2] / (1 << 16);
		event.data.mag.vect[0] = (float)sCalMag.mag_cal_q16[0] / (1 << 16);
		event.data.mag.vect[1] = (float)sCalMag.mag_cal_q16[1] / (1 << 16);
		event.data.mag.vect[2] = (float)sCalMag.mag_cal_q16[2] / (1 << 16);
		event.data.mag.accuracy_flag = sCalMag.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}

	/* 
	 * New GeoRV event 
	 */
	if(enabled_sensor_mask & (1 << SENSOR_GEORV)) {
		event.sensor	= INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR;
		event.timestamp = timestamp;
		event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
		event.data.quaternion.quat[0]  = (float)sGeoRV.georv_quat_q30[0] / (1 << 30);
		event.data.quaternion.quat[1]  = (float)sGeoRV.georv_quat_q30[1] / (1 << 30);
		event.data.quaternion.quat[2]  = (float)sGeoRV.georv_quat_q30[2] / (1 << 30);
		event.data.quaternion.quat[3]  = (float)sGeoRV.georv_quat_q30[3] / (1 << 30);
		event.data.quaternion.accuracy = (float)sGeoRV.georv_accuracy / (1 << 16);
		/* Report additionnal accuracy flag, being currently a copy of GYR accuracy flag (ACC flag could also be considered) */
		event.data.quaternion.accuracy_flag = sCalMag.accuracy_flag & DATA_ACCURACY_MASK;

		sensor_event(&event, NULL);
	}
}
#endif

int handle_command(enum DynProtocolEid eid, const DynProtocolEdata_t * edata, DynProtocolEdata_t * respdata)
{
	int rc, i;
	uint8_t whoami;
	const int sensor = edata->sensor_id;

	switch(eid) {

	case DYN_PROTOCOL_EID_GET_SW_REG:
		if(edata->d.command.regAddr == DYN_PROTOCOL_EREG_HANDSHAKE_SUPPORT)
			return uart_get_hw_flow_control_configuration(MAIN_UART_ID);
		return 0;

	case DYN_PROTOCOL_EID_SETUP:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command setup");
		/* Re-init the device */
		rc = icm20602_sensor_setup();
		rc += icm20602_sensor_configuration();
#if USE_AK09911_MAG
		if (ak09911_is_available)
			rc += ak09911_sensor_setup();
#endif
		/* Re-init algorithms */
		algorithms_init();
		apply_stored_offsets();
		sensor_configure_odr(period_us);
		/* Enable all sensors but.. */
		rc += sensor_control(1);
		/* .. no sensors are reporting on setup */
		enabled_sensor_mask = 0;
		return rc;

	case DYN_PROTOCOL_EID_WHO_AM_I:
		rc = inv_icm20602_get_whoami(&icm_device, &whoami);
		return (rc == 0) ? whoami : rc;

	case DYN_PROTOCOL_EID_RESET:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command reset");
		/* --- Cleanup --- */
		rc = sensor_control(0);
		store_offsets();
		/* Soft reset */
		rc += inv_icm20602_soft_reset(&icm_device);
#if USE_AK09911_MAG
		if(ak09911_is_available)
			rc += inv_ak0991x_soft_reset(&ak_device);
#endif
		/* --- Setup --- */
		/* Re-init the device */
		rc += icm20602_sensor_setup();
		rc += icm20602_sensor_configuration();
#if USE_AK09911_MAG
		if(ak09911_is_available)
			rc += ak09911_sensor_setup();
#endif
		/* Re-init algorithms */
		algorithms_init();
		apply_stored_offsets();
		sensor_configure_odr(period_us);
		/* Enable all sensor but.. */
		rc += sensor_control(1);
		/* All sensors stop reporting on reset */
		enabled_sensor_mask = 0;
		return rc;

	case DYN_PROTOCOL_EID_PING_SENSOR:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command ping(%s)", inv_sensor_2str(sensor));
		if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER)
				|| (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE)
				|| (sensor == INV_SENSOR_TYPE_ACCELEROMETER)
				|| (sensor == INV_SENSOR_TYPE_GYROSCOPE)
				|| (sensor == INV_SENSOR_TYPE_UNCAL_GYROSCOPE)
				|| (sensor == INV_SENSOR_TYPE_GAME_ROTATION_VECTOR)
				|| (sensor == INV_SENSOR_TYPE_GRAVITY)
				|| (sensor == INV_SENSOR_TYPE_LINEAR_ACCELERATION)
			) {
			return 0;
		} else if(sensor == INV_SENSOR_TYPE_PREDICTIVE_QUATERNION) {
			if (hmd_features_supported)
				return 0;
			else
				return INV_ERROR_BAD_ARG;
#if USE_AK09911_MAG
		} else if((sensor == INV_SENSOR_TYPE_RAW_MAGNETOMETER)
				|| (sensor == INV_SENSOR_TYPE_MAGNETOMETER)
				|| (sensor == INV_SENSOR_TYPE_UNCAL_MAGNETOMETER)
				|| (sensor == INV_SENSOR_TYPE_ROTATION_VECTOR)
				|| (sensor == INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR)
			) {
			if (ak09911_is_available)
				return 0;
			else
				return INV_ERROR_BAD_ARG;
#endif
		} else 
			return INV_ERROR_BAD_ARG;

	case DYN_PROTOCOL_EID_SELF_TEST:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command selft_test(%s)", inv_sensor_2str(sensor));
		if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER)
				|| (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE))
			return icm20602_run_selftest();
		else 
			return INV_ERROR_NIMPL;

	case DYN_PROTOCOL_EID_START_SENSOR:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command start(%s)", inv_sensor_2str(sensor));
		if (sensor > 0 && idd_sensortype_conversion(sensor) < SENSOR_MAX) {
			/* Sensor data will be notified */
			enabled_sensor_mask |= (1 << idd_sensortype_conversion(sensor));
			return 0;
		} else
			return INV_ERROR_NIMPL; /*this sensor is not supported*/

	case DYN_PROTOCOL_EID_STOP_SENSOR:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command stop(%s)", inv_sensor_2str(sensor));
		if (sensor > 0 && idd_sensortype_conversion(sensor) < SENSOR_MAX) {
			/* Sensor data will not be notified anymore */
			enabled_sensor_mask &= ~(1 << idd_sensortype_conversion(sensor));
			return 0;
		} else
			return INV_ERROR_NIMPL; /*this sensor is not supported*/

	case DYN_PROTOCOL_EID_SET_SENSOR_PERIOD:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command set_period(%d us)",edata->d.command.period);
		return sensor_configure_odr(edata->d.command.period);

	case DYN_PROTOCOL_EID_SET_SENSOR_CFG:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command set_sensor_config(%s)", inv_sensor_2str(sensor));
		if(edata->d.command.cfg.base.type == VSENSOR_CONFIG_TYPE_REFERENCE_FRAME) {
			float mmatrix[9];
			
			/* Ensure any float manipulation is word-aligned because of unsupported unaligned float access depending on CPU target */
			memcpy(mmatrix, edata->d.command.cfg.buffer, edata->d.command.cfg.size);
			
			if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER)
					|| (sensor == INV_SENSOR_TYPE_ACCELEROMETER)
					|| (sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE)
					|| (sensor == INV_SENSOR_TYPE_GYROSCOPE)
					|| (sensor == INV_SENSOR_TYPE_UNCAL_GYROSCOPE)) {
				for(i = 0; i < 9; ++i)
					cfg_mounting_matrix[i] = (int32_t)(mmatrix[i] * (1L << 30));
				return 0;
				}
#if USE_AK09911_MAG
			else if((sensor == INV_SENSOR_TYPE_RAW_MAGNETOMETER)
					|| (sensor == INV_SENSOR_TYPE_MAGNETOMETER)
					|| (sensor == INV_SENSOR_TYPE_UNCAL_MAGNETOMETER)) {
				for(i = 0; i < 9; ++i)
					cfg_mag_mounting_matrix[i] = (int32_t)(mmatrix[i] * (1L << 30));
				return 0;
				}
#endif
			else return INV_ERROR_BAD_ARG;
		}
		else
			return INV_ERROR_NIMPL;

	case DYN_PROTOCOL_EID_CLEANUP:
		INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: received command cleanup");
		rc = sensor_control(0);
		store_offsets();
		/* Soft reset */
		rc += inv_icm20602_soft_reset(&icm_device);
#if USE_AK09911_MAG
		if(ak09911_is_available)
			rc += inv_ak0991x_soft_reset(&ak_device);
#endif
		/* All sensors stop reporting on cleanup */
		enabled_sensor_mask = 0;
		return rc;

	default:
		return INV_ERROR_NIMPL;
	}
}

/*
 * IddWrapper protocol handler function
 *
 * Will dispatch command and send response back
 */
void iddwrapper_protocol_event_cb(
		enum DynProtocolEtype etype,
		enum DynProtocolEid eid,
		const DynProtocolEdata_t * edata,
		void * cookie
)
{
	(void)cookie;

	static DynProtocolEdata_t resp_edata; /* static to take on .bss */
	static uart_dma_tx_buffer_t * dma_buffer;
	DynProTransportUartFrame_t uart_frame;
	int timeout = 5000; /* us */;

	switch(etype) {
	case DYN_PROTOCOL_ETYPE_CMD:
		resp_edata.d.response.rc = handle_command(eid, edata, &resp_edata);

		/* send back response */
		while((uart_dma_tx_take_buffer(&dma_buffer) != 0) && (timeout > 0)) {
			inv_icm20602_sleep_us(10);
			timeout -= 10;
		}

		if (timeout <= 0) {
			/* if no available buffer, can't respond to the command */
			INV_MSG(INV_MSG_LEVEL_WARNING, "iddwrapper_protocol_event_cb: 'uart_dma_tx_take_buffer()' timeouted, response dropped");
			break;
		}

		/* get memory location for the iddwrapper protocol payload */
		if(DynProTransportUart_txAssignBuffer(&transport, &uart_frame,
				dma_buffer->data, sizeof(dma_buffer->data)) != 0) {
			goto error_dma_buffer;
		}
		if(DynProtocol_encodeResponse(&protocol, eid, &resp_edata,
				uart_frame.payload_data, uart_frame.max_payload_len, &uart_frame.payload_len) != 0) {
			goto error_dma_buffer;
		}
		if(DynProTransportUart_txEncodeFrame(&transport, &uart_frame) != 0) {
			goto error_dma_buffer;
		}
		/* respond to the command */
		dma_buffer->len = uart_frame.len;
		if(uart_dma_tx(MAIN_UART_ID, dma_buffer) != 0) {
			INV_MSG(INV_MSG_LEVEL_DEBUG, "iddwrapper_protocol_event_cb: 'uart_dma_tx()' returned error");
		}
		break;

error_dma_buffer:
		INV_MSG(INV_MSG_LEVEL_WARNING, "iddwrapper_protocol_event_cb: encode error, response dropped");
		uart_dma_tx_release_buffer(&dma_buffer);
		break;

	default:
		INV_MSG(INV_MSG_LEVEL_WARNING, "DeviceEmdWrapper: unexpected packet received. Ignored.");
		break; /* no suppose to happen */
	}
}

/*
 * IddWrapper transport handler function
 *
 * This function will:
 *  - feed the Dynamic protocol layer to analyse for incomming CMD packet
 *  - forward byte comming from transport layer to be send over uart to the host
 */
static void iddwrapper_transport_event_cb(enum DynProTransportEvent e,
	union DynProTransportEventData data, void * cookie)
{
	(void)cookie;

	int rc;

	switch(e) {
	case DYN_PRO_TRANSPORT_EVENT_ERROR:
		INV_MSG(INV_MSG_LEVEL_ERROR, "ERROR event with value %d received from IddWrapper transport", data.error);
		break;
	case DYN_PRO_TRANSPORT_EVENT_PKT_BYTE:
		/* Feed IddWrapperProtocol to look for packet */
		rc = DynProtocol_processPktByte(&protocol, data.pkt_byte);
		if(rc < 0) {
			INV_MSG(INV_MSG_LEVEL_DEBUG, "DynProtocol_processPktByte(%02x) returned %d", data.pkt_byte, rc);
		}
		break;
	default:
		break;
	}
}

/*
 * Convert sensor_event to VSensorData because dynamic protocol transports VSensorData
 */
static void convert_sensor_event_to_dyn_prot_data(const inv_sensor_event_t * event, VSensorDataAny * vsensor_data)
{
	vsensor_data->base.timestamp = event->timestamp;

	switch(event->sensor) {
	case DYN_PRO_SENSOR_TYPE_RESERVED:
		break;
	case DYN_PRO_SENSOR_TYPE_GRAVITY:
	case DYN_PRO_SENSOR_TYPE_LINEAR_ACCELERATION:
	case DYN_PRO_SENSOR_TYPE_ACCELEROMETER:
		inv_dc_float_to_sfix32(&event->data.acc.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.acc.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_GYROSCOPE:
		inv_dc_float_to_sfix32(&event->data.gyr.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.gyr.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_UNCAL_GYROSCOPE:
		inv_dc_float_to_sfix32(&event->data.gyr.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		inv_dc_float_to_sfix32(&event->data.gyr.bias[0], 3, 16, (int32_t *)&vsensor_data->data.u32[3]);
		vsensor_data->base.meta_data = event->data.gyr.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_PREDICTIVE_QUATERNION:
	case DYN_PRO_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		inv_dc_float_to_sfix32(&event->data.quaternion.quat[0], 4, 30, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.quaternion.accuracy_flag;
		break;
#if USE_AK09911_MAG
	case DYN_PRO_SENSOR_TYPE_MAGNETOMETER:
		inv_dc_float_to_sfix32(&event->data.mag.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		vsensor_data->base.meta_data = event->data.mag.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		inv_dc_float_to_sfix32(&event->data.mag.vect[0], 3, 16, (int32_t *)&vsensor_data->data.u32[0]);
		inv_dc_float_to_sfix32(&event->data.mag.bias[0], 3, 16, (int32_t *)&vsensor_data->data.u32[3]);
		vsensor_data->base.meta_data = event->data.mag.accuracy_flag;
		break;
	case DYN_PRO_SENSOR_TYPE_ROTATION_VECTOR:
	case DYN_PRO_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		inv_dc_float_to_sfix32(&event->data.quaternion.quat[0], 4, 30, (int32_t *)&vsensor_data->data.u32[0]);
		inv_dc_float_to_sfix32(&event->data.quaternion.accuracy, 1, 16, (int32_t *)&vsensor_data->data.u32[4]);
		vsensor_data->base.meta_data = event->data.quaternion.accuracy_flag;
		break;
#endif
	case DYN_PRO_SENSOR_TYPE_RAW_ACCELEROMETER:
	case DYN_PRO_SENSOR_TYPE_RAW_GYROSCOPE:
#if USE_AK09911_MAG
	case DYN_PRO_SENSOR_TYPE_RAW_MAGNETOMETER:
#endif
		vsensor_data->data.u32[0] = event->data.raw3d.vect[0];
		vsensor_data->data.u32[1] = event->data.raw3d.vect[1];
		vsensor_data->data.u32[2] = event->data.raw3d.vect[2];
		break;
	default:
		break;
	}
}

/*
 * Sleep implementation for ICM20602
 */
void inv_icm20602_sleep(int ms)
{
	delay_ms(ms);
}

void inv_icm20602_sleep_us(int us)
{
	delay_us(us);
}

/*
 * Callback that will be executed at each MAIN_UART interrupt
 */
static void ext_interrupt_main_uart_cb(void * context)
{
	uart_rx_context_t * ctx = (uart_rx_context_t *)context;
	
	/* 
	 * Analyze buffer provided in UART context 
	 */
	if (ctx->buf == 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "ext_interrupt_main_uart_cb: UART buffer empty");
		return;
	}
	/* 
	 * If number of bytes received on UART is 1,
	 * then we only have 1st byte of header, and we still need to receive 3 more bytes
	 * to be able to fully parse header 
	 */
	else if (ctx->buf->len == 1) {
		if (uart_dma_do_rx(ctx->uart, (uart_dma_rx_buffer_t *)ctx->buf, 3))
			INV_MSG(INV_MSG_LEVEL_ERROR, "ext_interrupt_main_uart_cb: 'uart_dma_do_rx()' returned error");
		return;
		/*
		 * IMPROVEMENT : here a timer should be fired so that if no DMA IRQ is fired
		 * after the time needed to receive 3 bytes on UART, then we had a serious issue
		 * and DMA transaction should be aborted 
		 */
	}
	/* 
	 * If number of bytes received on UART is 3,
	 * then we are now able to check header viability
	 * so that we can reprogram DMA to receive N bytes as indicated inside header 
	 */
	else if (ctx->buf->len == 4) {
		int len = DynProTransportUart_checkHeader_fromISR((uint8_t *)ctx->buf->data);
		if (len != -1) {
			if (uart_dma_do_rx(ctx->uart, (uart_dma_rx_buffer_t *)ctx->buf, len))
				INV_MSG(INV_MSG_LEVEL_ERROR, "ext_interrupt_main_uart_cb: 'uart_dma_do_rx()' returned error");
			/* 
			 * IMPROVEMENT : here a timer should be fired so that if no DMA IRQ is fired
			 * after the time needed to receive len bytes on UART, then we had a serious issue
			 * and DMA transaction should be aborted 
			 */
			return;
		}
	/* 
	* Full frame is now received on UART, ready to be parsed by CPU
	*/
	} else {
		if (uart_dma_rx_transfer_buffer((uart_dma_rx_buffer_t **)&ctx->buf))
			INV_MSG(INV_MSG_LEVEL_ERROR, "ext_interrupt_main_uart_cb: 'uart_dma_rx_transfer_buffer()' returned error");
		/* and reprogram DMA to wait for 1 byte for next packet frame */
		if (uart_dma_rx(ctx->uart))
			INV_MSG(INV_MSG_LEVEL_ERROR, "ext_interrupt_main_uart_cb: 'uart_dma_rx()' returned error");
		
		uart_dma_rx_buffer = ctx->buf;
		irq_event_main_uart = 1;
		return;
	}
}

/*
 * Callback called upon external interrupt line rising
 */
void ext_interrupt_cb(void * context, int int_num)
{
	(void)context;

	irq_from_device |= TO_MASK(int_num);
}

#if USE_AK09911_MAG
/*
 * Callback called at the end of the MAG capture
 */
void interrupt_timer_mag_ready_cb(void *context)
{
	(void)context;

	/* 
	 * Read timestamp from the timer dedicated to timestamping 
	 */
	uint64_t mag_timestamp = timer_get_counter(TIMEBASE_TIMER);

	if (!RINGBUFFER_FULL(&mag_timestamp_buffer)) {
		RINGBUFFER_PUSH(&mag_timestamp_buffer, &mag_timestamp);
	}

	irq_from_device |= TO_MASK(GPIO_SENSOR_IRQ_D5);
}

/*
 * Callback called to trigger MAG acquisition
 */
void interrupt_timer_start_mag_cb(void *context)
{
	(void)context;
	irq_start_mag_capture = 1;
}
#endif 

/******************************************************************************/
/* Low-level serial inteface function implementation                          */
/******************************************************************************/

/* Icm20602 Serif object definition for SPI/I2C **************************/

static int idd_io_hal_init(void)
{
#if SERIF_TYPE_SPI
	spi_master_init(SPI_NUM1, SPI_6MHZ);
#elif SERIF_TYPE_I2C
	/* 
	 * Configure I2C address with AD0 line connected to the sensor daugther board slot
	 * AD0=1 to be compatible with InvenSense AKM0991x daughter board internal configuration
	 */
	gpio_init_ad0_high();

	i2c_master_init();
#endif
	return 0;
}

static int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	(void)context;
#if SERIF_TYPE_SPI
	return spi_master_read_register(SPI_NUM1, reg, rlen, rbuffer);
#else /* SERIF_TYPE_I2C */
	return i2c_master_read_register(ICM_I2C_ADDR, reg, rlen, rbuffer);
#endif

}

static int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	(void)context;
#if SERIF_TYPE_SPI
	return spi_master_write_register(SPI_NUM1, reg, wlen, wbuffer);
#else /* SERIF_TYPE_I2C */
	return i2c_master_write_register(ICM_I2C_ADDR, reg, wlen, wbuffer);
#endif
}

#if USE_AK09911_MAG
/* Ak0991x Serif object definition for I2C ***************************************/

static int idd_io_hal_init_ak09911(void)
{
	i2c_master_init();
	return 0;
}

static int idd_io_hal_read_reg_ak09911(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	return i2c_master_read_register(AK_I2C_ADDR, reg, rlen, rbuffer);
}

static int idd_io_hal_write_reg_ak09911(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	return i2c_master_write_register(AK_I2C_ADDR, reg, wlen, wbuffer);
}
#endif

/*
 * Helper function to check RC value and block programm exectution
 */
static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}

#ifdef INV_MSG_ENABLE
/*
 * Printer function for message facility
 */
static void msg_printer(int level, const char * str, va_list ap)
{
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * ptr = out_str;
	const char * s[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if(idx >= (sizeof(out_str)))
		return;

	while(*ptr != '\0') {
		uart_putc(LOG_UART_ID, *ptr);
		++ptr;
	}
}
#endif
