/*
 * main.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

// Registers
#include "rcc_reg.h"
// Peripherals.
#include "exti.h"
#include "gpio.h"
#include "i2c_address.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Components.
#include "mma865xfc.h"
#include "mma865xfc_configuration.h"
#include "sensors_hw.h"
#include "sht3x.h"
// Utils.
#include "error.h"
#include "types.h"
// Middleware.
#include "analog.h"
#include "cli.h"
#include "gps.h"
#include "power.h"
#include "sigfox_ep_api.h"
#include "sigfox_types.h"
#include "sigfox_rc.h"
// Applicative.
#include "at.h"
#include "error_base.h"
#include "tkfx_flags.h"
#include "version.h"

/*** MAIN macros ***/

#ifdef TKFX_MODE_CAR
#define TKFX_MODE								0b00
#endif
#ifdef TKFX_MODE_BIKE
#define TKFX_MODE								0b01
#endif
#ifdef TKFX_MODE_HIKING
#define TKFX_MODE								0b10
#endif
// Sigfox payload lengths.
#define TKFX_SIGFOX_STARTUP_DATA_SIZE			8
#define TKFX_SIGFOX_GEOLOC_DATA_SIZE			11
#define TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE	3
#define TKFX_SIGFOX_MONITORING_DATA_SIZE		7
#define TKFX_SIGFOX_ERROR_STACK_DATA_SIZE		12
// Error values.
#define TKFX_ERROR_VALUE_ANALOG_16BITS			0xFFFF
#define TKFX_ERROR_VALUE_TEMPERATURE			0x7F
#define TKFX_ERROR_VALUE_HUMIDITY				0xFF
// Error stack message period.
#define TKFX_ERROR_STACK_PERIOD_SECONDS			86400
// Altitude stability filter.
#define TKFX_GEOLOC_TIMEOUT_SECONDS             180
#define TKFX_ALTITUDE_STABILITY_FILTER_MOVING	2
#define TKFX_ALTITUDE_STABILITY_FILTER_STOPPED	5

/*** MAIN structures ***/

/*******************************************************************/
typedef enum {
	TKFX_STATE_STARTUP,
	TKFX_STATE_WAKEUP,
	TKFX_STATE_MEASURE,
	TKFX_STATE_MODE_UPDATE,
	TKFX_STATE_MONITORING,
	TKFX_STATE_GEOLOC,
	TKFX_STATE_ERROR_STACK,
	TKFX_STATE_OFF,
	TKFX_STATE_SLEEP,
	TKFX_STATE_LAST
} TKFX_state_t;

/*******************************************************************/
typedef enum {
	TKFX_MODE_ACTIVE,
	TKFX_MODE_LOW_POWER,
	TKFX_MODE_LAST
} TKFX_mode_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned gps_backup_status : 1;
		unsigned accelerometer_status : 1;
		unsigned lse_status : 1;
		unsigned lsi_status : 1;
		unsigned moving_flag : 1;
		unsigned alarm_flag : 1;
		unsigned tracker_mode : 2;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
	uint8_t all;
} TKFX_status_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned unused : 5;
		unsigned geoloc_request : 1;
		unsigned monitoring_request : 1;
		unsigned por : 1;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
	uint8_t all;
} TKFX_flags_t;

/*******************************************************************/
typedef union {
	uint8_t frame[TKFX_SIGFOX_STARTUP_DATA_SIZE];
	struct {
		unsigned reset_reason : 8;
		unsigned major_version : 8;
		unsigned minor_version : 8;
		unsigned commit_index : 8;
		unsigned commit_id : 28;
		unsigned dirty_flag : 4;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} TKFX_sigfox_startup_data_t;

/*******************************************************************/
typedef union {
	uint8_t frame[TKFX_SIGFOX_MONITORING_DATA_SIZE];
	struct {
		unsigned tamb_degrees : 8;
		unsigned hamb_degrees : 8;
		unsigned vsrc_mv : 16;
		unsigned vstr_mv : 16;
		unsigned status : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} TKFX_sigfox_monitoring_data_t;

/*******************************************************************/
typedef union {
	uint8_t frame[TKFX_SIGFOX_GEOLOC_DATA_SIZE];
	struct {
		unsigned latitude_degrees : 8;
		unsigned latitude_minutes : 6;
		unsigned latitude_seconds : 17;
		unsigned latitude_north_flag : 1;
		unsigned longitude_degrees : 8;
		unsigned longitude_minutes : 6;
		unsigned longitude_seconds : 17;
		unsigned longitude_east_flag : 1;
		unsigned altitude_meters : 16;
		unsigned gps_fix_duration_seconds : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} TKFX_sigfox_geoloc_data_t;

/*******************************************************************/
typedef union {
    uint8_t frame[TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE];
    struct {
        unsigned gps_acquisition_status :8;
        unsigned gps_acquisition_duration_seconds :8;
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed));
} TKFX_sigfox_geoloc_timeout_data_t;

/*!******************************************************************
 * \struct TKFX_configuration_t
 * \brief Tracker configuration structure.
 *******************************************************************/
typedef struct {
	uint32_t start_detection_threshold_irq;
	uint32_t stop_detection_threshold_seconds;
	uint32_t moving_geoloc_period_seconds;
	uint32_t stopped_geoloc_period_seconds;
	uint32_t monitoring_period_seconds;
} TKFX_configuration_t;

#ifndef TKFX_MODE_CLI
/*******************************************************************/
typedef struct {
	// Global.
	TKFX_state_t state;
	TKFX_mode_t mode;
	TKFX_flags_t flags;
	// Tracker algorithm.
	volatile uint32_t start_detection_irq_count;
	volatile uint32_t last_motion_irq_time_seconds;
	uint32_t monitoring_next_time_seconds;
	uint32_t geoloc_next_time_seconds;
	uint32_t error_stack_next_time_seconds;
	// SW version.
	TKFX_sigfox_startup_data_t sigfox_startup_data;
	// Monitoring.
	TKFX_status_t status;
	uint8_t tamb_degrees;
	uint8_t hamb_percent;
	uint32_t vsrc_mv;
	uint32_t vstr_mv;
	TKFX_sigfox_monitoring_data_t sigfox_monitoring_data;
	// Geoloc.
	NEOM8X_position_t geoloc_position;
	TKFX_sigfox_geoloc_data_t sigfox_geoloc_data;
	TKFX_sigfox_geoloc_timeout_data_t sigfox_geoloc_timeout_data;
	// Error stack.
	uint8_t sigfox_error_stack_data[TKFX_SIGFOX_ERROR_STACK_DATA_SIZE];
} TKFX_context_t;
#endif

/*** MAIN global variables ***/

#ifndef TKFX_MODE_CLI
static TKFX_context_t tkfx_ctx;
#ifdef TKFX_MODE_CAR
static const TKFX_configuration_t TKFX_CONFIG = {0, 150, 300, 86400, 3600};
#endif
#ifdef TKFX_MODE_BIKE
static const TKFX_configuration_t TKFX_CONFIG = {5, 150, 300, 86400, 3600};
#endif
#ifdef TKFX_MODE_HIKING
static const TKFX_configuration_t TKFX_CONFIG = {5, 60, 600, 86400, 3600};
#endif
#endif

/*** MAIN functions ***/

/*******************************************************************/
static void _TKFX_rtc_wakeup_timer_irq_callback(void) {
#ifndef TKFX_MODE_CLI
    // Decrement IRQ count.
    if (tkfx_ctx.start_detection_irq_count > 0) {
        tkfx_ctx.start_detection_irq_count--;
    }
#endif
}

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_motion_irq_callback(void) {
	// Update variables.
	tkfx_ctx.start_detection_irq_count++;
	tkfx_ctx.last_motion_irq_time_seconds = RTC_get_uptime_seconds();
}
#endif

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_init_context(void) {
	// Init context.
	tkfx_ctx.state = TKFX_STATE_STARTUP;
	tkfx_ctx.mode = TKFX_MODE_ACTIVE;
	tkfx_ctx.flags.all = 0;
	tkfx_ctx.flags.por = 1;
	tkfx_ctx.status.all = 0;
	tkfx_ctx.status.tracker_mode = TKFX_MODE;
	tkfx_ctx.start_detection_irq_count = 0;
	tkfx_ctx.last_motion_irq_time_seconds = 0;
	tkfx_ctx.monitoring_next_time_seconds = TKFX_CONFIG.monitoring_period_seconds;
	tkfx_ctx.geoloc_next_time_seconds = TKFX_CONFIG.stopped_geoloc_period_seconds;
	tkfx_ctx.error_stack_next_time_seconds = 0;
	// Set motion interrupt callback address.
	SENSORS_HW_set_accelerometer_irq_callback(&_TKFX_motion_irq_callback);
}
#endif

/*******************************************************************/
static void _TKFX_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
#ifndef TKFX_MODE_DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init power module and clock tree.
	PWR_init();
	rcc_status = RCC_init(NVIC_PRIORITY_CLOCK);
	RCC_stack_error(ERROR_BASE_RCC);
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
	// Start independent watchdog.
#ifndef TKFX_MODE_DEBUG
	iwdg_status = IWDG_init();
	IWDG_stack_error(ERROR_BASE_IWDG);
	IWDG_reload();
#endif
	// High speed oscillator.
	rcc_status = RCC_switch_to_hsi();
	RCC_stack_error(ERROR_BASE_RCC);
	// Calibrate clocks.
	rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
	RCC_stack_error(ERROR_BASE_RCC);
	// Init RTC.
	rtc_status = RTC_init(&_TKFX_rtc_wakeup_timer_irq_callback, NVIC_PRIORITY_RTC);
	RTC_stack_error(ERROR_BASE_RTC);
	// Init delay timer.
	LPTIM_init(NVIC_PRIORITY_DELAY);
	// Init components.
	POWER_init();
}

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_send_sigfox_message(SIGFOX_EP_API_application_message_t* application_message) {
	// Local variables.
	SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
	SIGFOX_EP_API_config_t lib_config;
	// Disable motion interrupts.
	SENSORS_HW_disable_accelerometer_interrupt();
	// Library configuration.
	lib_config.rc = &SIGFOX_RC1;
	// Open library.
	sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
	SIGFOX_EP_API_stack_error();
	if (sigfox_ep_api_status == SIGFOX_EP_API_SUCCESS) {
		// Send message.
		sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(application_message);
		SIGFOX_EP_API_stack_error();
	}
	// Close library.
	sigfox_ep_api_status = SIGFOX_EP_API_close();
	SIGFOX_EP_API_stack_error();
	// Re-enable motion interrupts if enabled.
	if (tkfx_ctx.status.accelerometer_status != 0) {
		// Enable interrupt.
	    SENSORS_HW_enable_accelerometer_interrupt();
	}
}
#endif

#ifndef TKFX_MODE_CLI
/*******************************************************************/
int main (void) {
	// Init board.
	_TKFX_init_context();
	_TKFX_init_hw();
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	ANALOG_status_t analog_status = ANALOG_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	MMA865XFC_status_t mma865xfc_status = MMA865XFC_SUCCESS;
	GPS_status_t gps_status = GPS_SUCCESS;
	GPS_acquisition_status_t gps_acquisition_status = GPS_ACQUISITION_SUCCESS;
	uint32_t geoloc_fix_duration_seconds = 0;
	SIGFOX_EP_API_application_message_t application_message;
	ERROR_code_t error_code = 0;
	uint8_t idx = 0;
	int32_t generic_s32_1 = 0;
	int32_t generic_s32_2 = 0;
	uint8_t generic_u8;
	uint32_t generic_u32 = 0;
	// Application message default parameters.
	application_message.common_parameters.number_of_frames = 3;
	application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
	application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
#ifdef BIDIRECTIONAL
	application_message.bidirectional_flag = 0;
#endif
	application_message.ul_payload = SFX_NULL;
	application_message.ul_payload_size_bytes = 0;
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (tkfx_ctx.state) {
		case TKFX_STATE_STARTUP:
			IWDG_reload();
			// Fill reset reason and software version.
			tkfx_ctx.sigfox_startup_data.reset_reason = ((RCC -> CSR) >> 24) & 0xFF;
			tkfx_ctx.sigfox_startup_data.major_version = GIT_MAJOR_VERSION;
			tkfx_ctx.sigfox_startup_data.minor_version = GIT_MINOR_VERSION;
			tkfx_ctx.sigfox_startup_data.commit_index = GIT_COMMIT_INDEX;
			tkfx_ctx.sigfox_startup_data.commit_id = GIT_COMMIT_ID;
			tkfx_ctx.sigfox_startup_data.dirty_flag = GIT_DIRTY_FLAG;
			// Send SW version frame.
			application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
			application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_startup_data.frame);
			application_message.ul_payload_size_bytes = TKFX_SIGFOX_STARTUP_DATA_SIZE;
			_TKFX_send_sigfox_message(&application_message);
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_ERROR_STACK;
			break;
		case TKFX_STATE_WAKEUP:
			IWDG_reload();
			// Calibrate clocks.
			rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
			RCC_stack_error(ERROR_BASE_RCC);
			// Reset GPS status for mode update.
			gps_acquisition_status = GPS_ACQUISITION_SUCCESS;
			// Compute next state.
			tkfx_ctx.state = ((tkfx_ctx.flags.monitoring_request != 0) || (tkfx_ctx.flags.geoloc_request != 0)) ? TKFX_STATE_MEASURE : TKFX_STATE_MODE_UPDATE;
			break;
		case TKFX_STATE_MEASURE:
			IWDG_reload();
			// Get temperature from SHT30.
			power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
			POWER_stack_error(ERROR_BASE_POWER);
			sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30, &generic_s32_1, &generic_s32_2);
			SHT3X_stack_error(ERROR_BASE_SHT30);
			power_status = POWER_disable(POWER_DOMAIN_SENSORS);
			POWER_stack_error(ERROR_BASE_POWER);
			// Reset data.
			tkfx_ctx.tamb_degrees = TKFX_ERROR_VALUE_TEMPERATURE;
			tkfx_ctx.hamb_percent = TKFX_ERROR_VALUE_HUMIDITY;
			if (sht3x_status == SHT3X_SUCCESS) {
			    // Convert temperature.
                math_status = MATH_integer_to_signed_magnitude(generic_s32_1, (MATH_U8_SIZE_BITS - 1), &generic_u32);
                MATH_stack_error(ERROR_BASE_MATH);
                if (math_status == MATH_SUCCESS) {
                    tkfx_ctx.tamb_degrees = (uint8_t) generic_u32;
                }
				tkfx_ctx.hamb_percent = (uint8_t) generic_s32_2;
			}
			// Reset data.
            tkfx_ctx.vsrc_mv = TKFX_ERROR_VALUE_ANALOG_16BITS;
            tkfx_ctx.vstr_mv = TKFX_ERROR_VALUE_ANALOG_16BITS;
			// Get voltages measurements.
			power_status = POWER_enable(POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
			POWER_stack_error(ERROR_BASE_POWER);
			analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VSRC_MV, &generic_s32_1);
			ANALOG_stack_error(ERROR_BASE_ANALOG);
			if (analog_status == ANALOG_SUCCESS) {
			    tkfx_ctx.vsrc_mv = (uint32_t) generic_s32_1;
			}
			analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VSTR_MV, &generic_s32_1);
            ANALOG_stack_error(ERROR_BASE_ANALOG);
            if (analog_status == ANALOG_SUCCESS) {
                tkfx_ctx.vstr_mv = (uint32_t) generic_s32_1;
            }
			power_status = POWER_disable(POWER_DOMAIN_ANALOG);
			POWER_stack_error(ERROR_BASE_POWER);
			// Compute next state.
			if (tkfx_ctx.flags.monitoring_request != 0) {
				tkfx_ctx.state = TKFX_STATE_MONITORING;
			}
			else {
				if (tkfx_ctx.flags.geoloc_request != 0) {
					tkfx_ctx.state = TKFX_STATE_GEOLOC;
				}
				else {
					tkfx_ctx.state = TKFX_STATE_MODE_UPDATE;
				}
			}
			break;
		case TKFX_STATE_MONITORING:
			IWDG_reload();
			// Update GPS backup  status.
			tkfx_ctx.status.gps_backup_status = GPS_get_backup_voltage();
			// Get clocks status.
			rcc_status = RCC_get_status(RCC_CLOCK_LSI, &generic_u8);
			RCC_stack_error(ERROR_BASE_RCC);
			tkfx_ctx.status.lsi_status = (generic_u8 == 0) ? 0b0 : 0b1;
			rcc_status = RCC_get_status(RCC_CLOCK_LSE, &generic_u8);
			RCC_stack_error(ERROR_BASE_RCC);
			tkfx_ctx.status.lse_status = (generic_u8 == 0) ? 0b0 : 0b1;
			// Build Sigfox frame.
			tkfx_ctx.sigfox_monitoring_data.tamb_degrees = tkfx_ctx.tamb_degrees;
			tkfx_ctx.sigfox_monitoring_data.hamb_degrees = tkfx_ctx.hamb_percent;
			tkfx_ctx.sigfox_monitoring_data.vsrc_mv = tkfx_ctx.vsrc_mv;
			tkfx_ctx.sigfox_monitoring_data.vstr_mv = tkfx_ctx.vstr_mv;
			tkfx_ctx.sigfox_monitoring_data.status = tkfx_ctx.status.all;
			// Send uplink monitoring frame.
			application_message.common_parameters.ul_bit_rate = (tkfx_ctx.status.alarm_flag == 0) ? SIGFOX_UL_BIT_RATE_600BPS : SIGFOX_UL_BIT_RATE_100BPS;
			application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_monitoring_data.frame);
			application_message.ul_payload_size_bytes = TKFX_SIGFOX_MONITORING_DATA_SIZE;
			_TKFX_send_sigfox_message(&application_message);
			// Reset flag and timer.
			tkfx_ctx.flags.monitoring_request = 0;
			// Change error value for mode update.
			if (tkfx_ctx.vstr_mv == TKFX_ERROR_VALUE_ANALOG_16BITS) {
				tkfx_ctx.vstr_mv = 0;
			}
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_ERROR_STACK;
			break;
		case TKFX_STATE_ERROR_STACK:
			// Check period.
			if (RTC_get_uptime_seconds() >= tkfx_ctx.error_stack_next_time_seconds) {
				// Import Sigfox library error stack.
				ERROR_import_sigfox_stack();
				// Check stack.
				if (ERROR_stack_is_empty() == 0) {
					// Read error stack.
					for (idx=0 ; idx<(TKFX_SIGFOX_ERROR_STACK_DATA_SIZE >> 1) ; idx++) {
						error_code = ERROR_stack_read();
						tkfx_ctx.sigfox_error_stack_data[(idx << 1) + 0] = (uint8_t) ((error_code >> 8) & 0x00FF);
						tkfx_ctx.sigfox_error_stack_data[(idx << 1) + 1] = (uint8_t) ((error_code >> 0) & 0x00FF);
					}
					// Update next time.
					tkfx_ctx.error_stack_next_time_seconds = RTC_get_uptime_seconds() + TKFX_ERROR_STACK_PERIOD_SECONDS;
					// Send error stack frame.
					application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
					application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_error_stack_data);
					application_message.ul_payload_size_bytes = TKFX_SIGFOX_ERROR_STACK_DATA_SIZE;
					_TKFX_send_sigfox_message(&application_message);
				}
			}
			// Compute next state.
			tkfx_ctx.state = (tkfx_ctx.flags.geoloc_request != 0) ? TKFX_STATE_GEOLOC : TKFX_STATE_MODE_UPDATE;
			break;
		case TKFX_STATE_GEOLOC:
			IWDG_reload();
			// Enable backup if possible.
			if (tkfx_ctx.mode == TKFX_MODE_ACTIVE) {
				gps_status = GPS_set_backup_voltage(1);
				GPS_stack_error(ERROR_BASE_GPS);
			}
			// Configure altitude stability filter.
			generic_u8 = (tkfx_ctx.status.moving_flag == 0) ? TKFX_ALTITUDE_STABILITY_FILTER_STOPPED : TKFX_ALTITUDE_STABILITY_FILTER_MOVING;
			// Reset fix duration.
			geoloc_fix_duration_seconds = 0;
			// Pre-check storage voltage.
			if (tkfx_ctx.vstr_mv > TKFX_ACTIVE_MODE_VSTR_MIN_MV) {
			    // Turn analog front-end to monitor storage element voltage.
                power_status = POWER_enable(POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_SLEEP);
                POWER_stack_error(ERROR_BASE_POWER);
                // Get position from GPS.
				power_status = POWER_enable(POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
				POWER_stack_error(ERROR_BASE_POWER);
				gps_status = GPS_get_position(&tkfx_ctx.geoloc_position, generic_u8, TKFX_GEOLOC_TIMEOUT_SECONDS, &geoloc_fix_duration_seconds, &gps_acquisition_status);
				GPS_stack_error(ERROR_BASE_GPS);
				power_status = POWER_disable(POWER_DOMAIN_GPS);
				POWER_stack_error(ERROR_BASE_POWER);
				power_status = POWER_disable(POWER_DOMAIN_ANALOG);
                POWER_stack_error(ERROR_BASE_POWER);
			}
			else {
			    gps_acquisition_status = GPS_ACQUISITION_ERROR_VSTR_THRESHOLD;
			}
			// Compute bit rate according to tracker motion state.
#ifdef TKFX_MODE_HIKING
			application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#else
			application_message.common_parameters.ul_bit_rate = (tkfx_ctx.status.moving_flag == 0) ? SIGFOX_UL_BIT_RATE_100BPS : SIGFOX_UL_BIT_RATE_600BPS;
#endif
			// Build Sigfox frame.
			if (gps_acquisition_status == GPS_ACQUISITION_SUCCESS) {
				tkfx_ctx.sigfox_geoloc_data.latitude_degrees = tkfx_ctx.geoloc_position.lat_degrees;
				tkfx_ctx.sigfox_geoloc_data.latitude_minutes = tkfx_ctx.geoloc_position.lat_minutes;
				tkfx_ctx.sigfox_geoloc_data.latitude_seconds = tkfx_ctx.geoloc_position.lat_seconds;
				tkfx_ctx.sigfox_geoloc_data.latitude_north_flag = tkfx_ctx.geoloc_position.lat_north_flag;
				tkfx_ctx.sigfox_geoloc_data.longitude_degrees = tkfx_ctx.geoloc_position.long_degrees;
				tkfx_ctx.sigfox_geoloc_data.longitude_minutes = tkfx_ctx.geoloc_position.long_minutes;
				tkfx_ctx.sigfox_geoloc_data.longitude_seconds = tkfx_ctx.geoloc_position.long_seconds;
				tkfx_ctx.sigfox_geoloc_data.longitude_east_flag = tkfx_ctx.geoloc_position.long_east_flag;
				tkfx_ctx.sigfox_geoloc_data.altitude_meters = tkfx_ctx.geoloc_position.altitude;
				tkfx_ctx.sigfox_geoloc_data.gps_fix_duration_seconds = geoloc_fix_duration_seconds;
				// Update message parameters.
				application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_geoloc_data.frame);
				application_message.ul_payload_size_bytes = TKFX_SIGFOX_GEOLOC_DATA_SIZE;
			}
			else {
				tkfx_ctx.sigfox_geoloc_timeout_data.gps_acquisition_status = gps_acquisition_status;
				tkfx_ctx.sigfox_geoloc_timeout_data.gps_acquisition_duration_seconds = geoloc_fix_duration_seconds;
				// Update message parameters.
				application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_geoloc_timeout_data.frame);
				application_message.ul_payload_size_bytes = TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE;
			}
			// Send uplink geolocation frame.
			_TKFX_send_sigfox_message(&application_message);
			// Reset flag and timer.
			tkfx_ctx.flags.geoloc_request = 0;
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_MODE_UPDATE;
			break;
		case TKFX_STATE_MODE_UPDATE:
			IWDG_reload();
			// Perform measurements.
			power_status = POWER_enable(POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
			POWER_stack_error(ERROR_BASE_POWER);
			analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VSTR_MV, &generic_s32_1);
            ANALOG_stack_error(ERROR_BASE_ANALOG);
			power_status = POWER_disable(POWER_DOMAIN_ANALOG);
			POWER_stack_error(ERROR_BASE_POWER);
			// Read storage voltage.
			tkfx_ctx.vstr_mv = (analog_status == ANALOG_SUCCESS) ? generic_s32_1 : 0;
			// Check storage voltage.
			tkfx_ctx.mode = ((tkfx_ctx.vstr_mv < TKFX_ACTIVE_MODE_VSTR_MIN_MV) || (gps_acquisition_status == GPS_ACQUISITION_ERROR_VSTR_THRESHOLD)) ? TKFX_MODE_LOW_POWER : TKFX_MODE_ACTIVE;
			// Configure accelerometer according to mode.
			if ((tkfx_ctx.mode == TKFX_MODE_ACTIVE) && ((tkfx_ctx.status.accelerometer_status == 0) || (tkfx_ctx.flags.por != 0))) {
				// Active mode.
				power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
				POWER_stack_error(ERROR_BASE_POWER);
				mma865xfc_status = MMA865XFC_write_configuration(I2C_ADDRESS_MMA8653FC, &(MMA865XFC_ACTIVE_CONFIGURATION[0]), MMA865XFC_ACTIVE_CONFIGURATION_SIZE);
				MMA865XFC_stack_error(ERROR_BASE_MMA8653FC);
				power_status = POWER_disable(POWER_DOMAIN_SENSORS);
				POWER_stack_error(ERROR_BASE_POWER);
				// Enable interrupt.
				SENSORS_HW_enable_accelerometer_interrupt();
				// Update status.
				tkfx_ctx.status.accelerometer_status = 1;
			}
			if ((tkfx_ctx.mode == TKFX_MODE_LOW_POWER) && ((tkfx_ctx.status.accelerometer_status != 0) || (tkfx_ctx.flags.por != 0))) {
				// Sleep mode.
				power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
				POWER_stack_error(ERROR_BASE_POWER);
				mma865xfc_status = MMA865XFC_write_configuration(I2C_ADDRESS_MMA8653FC, &(MMA865XFC_SLEEP_CONFIGURATION[0]), MMA865XFC_SLEEP_CONFIGURATION_SIZE);
				MMA865XFC_stack_error(ERROR_BASE_MMA8653FC);
				power_status = POWER_disable(POWER_DOMAIN_SENSORS);
				POWER_stack_error(ERROR_BASE_POWER);
				// Disable interrupt.
				SENSORS_HW_disable_accelerometer_interrupt();
				// Update status.
				tkfx_ctx.status.accelerometer_status = 0;
			}
#ifdef HW1_1
			// Disable GPS backup in low power mode.
			if (tkfx_ctx.mode == TKFX_MODE_LOW_POWER) {
			    gps_status = GPS_set_backup_voltage(0);
			    GPS_stack_error(ERROR_BASE_GPS);
			}
#endif
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_OFF;
			break;
		case TKFX_STATE_OFF:
			IWDG_reload();
			// Clear POR flag.
			tkfx_ctx.flags.por = 0;
			// Reset IRQ count.
			tkfx_ctx.start_detection_irq_count = 0;
			// Enter sleep mode.
			tkfx_ctx.state = TKFX_STATE_SLEEP;
			break;
		case TKFX_STATE_SLEEP:
			// Enter stop mode.
			IWDG_reload();
			PWR_enter_stop_mode();
			IWDG_reload();
			// Periodic monitoring.
			if (RTC_get_uptime_seconds() >= tkfx_ctx.monitoring_next_time_seconds) {
				// Compute next time.
				tkfx_ctx.monitoring_next_time_seconds += TKFX_CONFIG.monitoring_period_seconds;
				// Update requests.
				tkfx_ctx.flags.monitoring_request = 1;
				// Update status.
				tkfx_ctx.status.alarm_flag = 0;
				// Turn tracker on to send monitoring message.
				tkfx_ctx.state = TKFX_STATE_WAKEUP;
			}
			// Periodic geolocation.
			if (RTC_get_uptime_seconds() >= tkfx_ctx.geoloc_next_time_seconds) {
				// Compute next time.
				generic_u32 = (tkfx_ctx.status.moving_flag == 0) ? TKFX_CONFIG.stopped_geoloc_period_seconds : TKFX_CONFIG.moving_geoloc_period_seconds;
				tkfx_ctx.geoloc_next_time_seconds += generic_u32;
				// Check mode.
				if (tkfx_ctx.mode == TKFX_MODE_ACTIVE) {
					// Update requests.
					tkfx_ctx.flags.geoloc_request = 1;
					// Update status.
					tkfx_ctx.status.alarm_flag = 0;
					// Turn tracker on to perform periodic geolocation.
					tkfx_ctx.state = TKFX_STATE_WAKEUP;
				}
			}
			// Start detection.
			if ((tkfx_ctx.status.moving_flag == 0) && (tkfx_ctx.start_detection_irq_count > TKFX_CONFIG.start_detection_threshold_irq) && (tkfx_ctx.mode == TKFX_MODE_ACTIVE)) {
				// Update requests.
				tkfx_ctx.flags.monitoring_request = 1;
				// Update status.
				tkfx_ctx.status.moving_flag = 1;
				tkfx_ctx.status.alarm_flag = 1;
				// Always reset timers on event.
				tkfx_ctx.monitoring_next_time_seconds = RTC_get_uptime_seconds() + TKFX_CONFIG.monitoring_period_seconds;
				tkfx_ctx.geoloc_next_time_seconds = RTC_get_uptime_seconds() + TKFX_CONFIG.moving_geoloc_period_seconds;
				// Turn tracker on to send start alarm.
				tkfx_ctx.state = TKFX_STATE_WAKEUP;
			}
			else {
				// Stop detection.
				if ((tkfx_ctx.status.moving_flag != 0) && (RTC_get_uptime_seconds() >= (tkfx_ctx.last_motion_irq_time_seconds + TKFX_CONFIG.stop_detection_threshold_seconds)) && (tkfx_ctx.mode == TKFX_MODE_ACTIVE)) {
					// Update requests.
					tkfx_ctx.flags.monitoring_request = 1;
					tkfx_ctx.flags.geoloc_request = 1;
					// Update status.
					tkfx_ctx.status.moving_flag = 0;
					tkfx_ctx.status.alarm_flag = 1;
					// Always reset timers on event.
					tkfx_ctx.monitoring_next_time_seconds = RTC_get_uptime_seconds() + TKFX_CONFIG.monitoring_period_seconds;
					tkfx_ctx.geoloc_next_time_seconds = RTC_get_uptime_seconds() + TKFX_CONFIG.stopped_geoloc_period_seconds;
					// Turn tracker on to send stop alarm.
					tkfx_ctx.state = TKFX_STATE_WAKEUP;
				}
			}
			break;
		default:
			// Unknown state.
			tkfx_ctx.state = TKFX_STATE_OFF;
			break;
		}
	}
	return 0;
}
#endif

#ifdef TKFX_MODE_CLI
/*******************************************************************/
int main (void) {
	// Local variables.
	CLI_status_t cli_status = CLI_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	MMA865XFC_status_t mma865xfc_status = MMA865XFC_SUCCESS;
    // Init board.
    _TKFX_init_hw();
	// Configure accelerometer.
	power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
	POWER_stack_error(ERROR_BASE_POWER);
	mma865xfc_status = MMA865XFC_write_configuration(I2C_ADDRESS_MMA8653FC, &(MMA865XFC_ACTIVE_CONFIGURATION[0]), MMA865XFC_ACTIVE_CONFIGURATION_SIZE);
	MMA865XFC_stack_error(ERROR_BASE_MMA8653FC);
	power_status = POWER_disable(POWER_DOMAIN_SENSORS);
	POWER_stack_error(ERROR_BASE_POWER);
	// Init command line interface.
    cli_status = CLI_init();
    CLI_stack_error(ERROR_BASE_CLI);
	// Main loop.
	while (1) {
		// Enter sleep mode.
		IWDG_reload();
		PWR_enter_sleep_mode();
		IWDG_reload();
		// Process command line interface.
        cli_status = CLI_process();
        CLI_stack_error(ERROR_BASE_CLI);
	}
	return 0;
}
#endif

