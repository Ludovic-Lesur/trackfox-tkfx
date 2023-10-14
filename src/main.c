/*
 * main.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

// Registers
#include "rcc_reg.h"
// Peripherals.
#include "adc.h"
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Components.
#include "mma8653fc.h"
#include "neom8n.h"
#include "power.h"
#include "sht3x.h"
// Utils.
#include "types.h"
// Sigfox.
#include "sigfox_ep_api.h"
#include "sigfox_types.h"
#include "sigfox_rc.h"
// Applicative.
#include "at.h"
#include "error.h"
#include "mode.h"
#include "version.h"

/*** MAIN macros ***/

// Sigfox payload lengths.
#define TKFX_SIGFOX_STARTUP_DATA_SIZE			8
#define TKFX_SIGFOX_GEOLOC_DATA_SIZE			11
#define TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE	3
#define TKFX_SIGFOX_DOWNLINK_DATA_SIZE			8
#define TKFX_SIGFOX_MONITORING_DATA_SIZE		9
#define TKFX_SIGFOX_ERROR_STACK_DATA_SIZE		12
// Error values.
#define TKFX_ERROR_VALUE_ANALOG_12BITS			0xFFF
#define TKFX_ERROR_VALUE_ANALOG_16BITS			0xFFFF
#define TKFX_ERROR_VALUE_TEMPERATURE			0x7F
#define TKFX_ERROR_VALUE_HUMIDITY				0xFF

/*** MAIN structures ***/

/*******************************************************************/
typedef enum {
	TKFX_STATE_STARTUP,
	TKFX_STATE_WAKEUP,
	TKFX_STATE_MODE_UPDATE,
	TKFX_STATE_ACCELERO,
	TKFX_STATE_MEASURE,
	TKFX_STATE_MONITORING,
	TKFX_STATE_GEOLOC_CHECK,
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
		unsigned tmcu_degrees : 8;
		unsigned vsrc_mv : 16;
		unsigned vcap_mv : 12;
		unsigned vmcu_mv : 12;
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
		unsigned error_code : 16;
		unsigned fix_duration_seconds : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} TKFX_sigfox_geoloc_timeout_data_t;

#if (defined SSM) || (defined PM)
/*******************************************************************/
typedef struct {
	// Global.
	TKFX_state_t state;
	TKFX_mode_t mode;
	TKFX_flags_t flags;
	const TKFX_configuration_t* config;
	// Wake-up management.
#ifdef SSM
	uint32_t start_detection_irq_count;
	uint32_t stop_detection_timer_seconds;
	uint32_t keep_alive_timer_seconds;
#endif
	uint32_t geoloc_timer_seconds;
	// SW version.
	TKFX_sigfox_startup_data_t sigfox_startup_data;
	// Monitoring.
	TKFX_status_t status;
	uint8_t tamb_degrees;
	uint8_t hamb_percent;
	uint8_t tmcu_degrees;
	uint32_t vsrc_mv;
	uint32_t vcap_mv;
	uint32_t vmcu_mv;
	TKFX_sigfox_monitoring_data_t sigfox_monitoring_data;
	// Geoloc.
	NEOM8N_position_t geoloc_position;
	uint32_t geoloc_fix_duration_seconds;
	TKFX_sigfox_geoloc_data_t sigfox_geoloc_data;
	TKFX_sigfox_geoloc_timeout_data_t sigfox_geoloc_timeout_data;
	// Error stack.
	uint8_t sigfox_error_stack_data[TKFX_SIGFOX_ERROR_STACK_DATA_SIZE];
	// Sigfox.
	uint8_t sigfox_downlink_data[TKFX_SIGFOX_DOWNLINK_DATA_SIZE];
} TKFX_context_t;
#endif

/*** MAIN global variables ***/

#if (defined SSM) || (defined PM)
static TKFX_context_t tkfx_ctx;
#endif

/*** MAIN functions ***/

#if (defined SSM) || (defined PM)
/*******************************************************************/
static void _TKFX_init_context(void) {
	// Init context.
	tkfx_ctx.state = TKFX_STATE_STARTUP;
	tkfx_ctx.mode = TKFX_MODE_ACTIVE;
	tkfx_ctx.flags.all = 0;
	tkfx_ctx.flags.por = 1;
	tkfx_ctx.config = &TKFX_CONFIG;
	tkfx_ctx.status.all = 0;
#ifdef SSM
	tkfx_ctx.status.tracker_mode = 0b00;
	tkfx_ctx.start_detection_irq_count = 0;
	tkfx_ctx.stop_detection_timer_seconds = 0;
	tkfx_ctx.keep_alive_timer_seconds = 0;
#endif
	tkfx_ctx.geoloc_timer_seconds = 0;
#ifdef PM
	tkfx_ctx.status.tracker_mode = 0b01;
#endif
	tkfx_ctx.geoloc_fix_duration_seconds = 0;
}
#endif

/*******************************************************************/
static void _TKFX_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	// Init power module and clock tree.
	PWR_init();
	rcc_status = RCC_init();
	RCC_stack_error();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
	// Start independent watchdog.
#ifndef DEBUG
	iwdg_status = IWDG_init();
	IWDG_stack_error();
	IWDG_reload();
#endif
	// High speed oscillator.
	rcc_status = RCC_switch_to_hsi();
	RCC_stack_error();
	// Init RTC.
	rtc_status = RTC_init();
	RTC_stack_error();
	// Init delay timer.
	LPTIM1_init();
	// Init components.
	POWER_init();
#ifdef SSM
	MMA8653FC_init();
#endif
#ifdef ATM
	// Applicative layers.
	AT_init();
#endif
}

#if (defined SSM) || (defined PM)
/*******************************************************************/
static void _TKFX_send_sigfox_message(SIGFOX_EP_API_application_message_t* application_message) {
	// Local variables.
	SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
	SIGFOX_EP_API_config_t lib_config;
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
}
#endif

#if (defined SSM) || (defined PM)
/*******************************************************************/
int main (void) {
	// Init board.
	_TKFX_init_context();
	_TKFX_init_hw();
	// Local variables.
	POWER_status_t power_status = POWER_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	MMA8653FC_status_t mma8653fc_status = MMA8653FC_SUCCESS;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	SIGFOX_EP_API_application_message_t application_message;
	ERROR_code_t error_code = 0;
	uint8_t idx = 0;
	int8_t temperature = 0;
	uint8_t generic_data_u8;
	uint32_t generic_data_u32 = 0;
	// Application message default parameters.
	application_message.common_parameters.number_of_frames = 3;
	application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
	application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
	application_message.bidirectional_flag = 0;
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
			application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
			application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_startup_data.frame);
			application_message.ul_payload_size_bytes = TKFX_SIGFOX_STARTUP_DATA_SIZE;
			_TKFX_send_sigfox_message(&application_message);
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_MEASURE;
			break;
		case TKFX_STATE_WAKEUP:
			IWDG_reload();
#ifdef SSM
			// Disable accelerometer interrupt.
			MMA8653FC_clear_motion_interrupt_flag();
			MMA8653_disable_motion_interrupt();
#endif
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_MEASURE;
			break;
		case TKFX_STATE_MEASURE:
			IWDG_reload();
			// Get temperature from SHT30.
			power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
			POWER_stack_error();
			sht3x_status = SHT3X_perform_measurements(SHT3X_I2C_ADDRESS);
			SHT3X_stack_error();
			power_status = POWER_disable(POWER_DOMAIN_SENSORS);
			POWER_stack_error();
			// Reset data.
			tkfx_ctx.tamb_degrees = TKFX_ERROR_VALUE_TEMPERATURE;
			tkfx_ctx.hamb_percent = TKFX_ERROR_VALUE_HUMIDITY;
			if (sht3x_status == SHT3X_SUCCESS) {
				// Read temperature.
				sht3x_status = SHT3X_get_temperature(&temperature);
				SHT3X_stack_error();
				if (sht3x_status == SHT3X_SUCCESS) {
					math_status = MATH_int32_to_signed_magnitude((int32_t) temperature, 7, &generic_data_u32);
					MATH_stack_error();
					if (math_status == MATH_SUCCESS) {
						tkfx_ctx.tamb_degrees = (uint8_t) generic_data_u32;
					}
				}
				// Read humidity.
				sht3x_status = SHT3X_get_humidity(&generic_data_u8);
				SHT3X_stack_error();
				if (sht3x_status == SHT3X_SUCCESS) {
					tkfx_ctx.hamb_percent = generic_data_u8;
				}
			}
			// Get voltages measurements.
			power_status = POWER_enable(POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
			POWER_stack_error();
			adc1_status = ADC1_perform_measurements();
			ADC1_stack_error();
			power_status = POWER_disable(POWER_DOMAIN_ANALOG);
			POWER_stack_error();
			// Reset data.
			tkfx_ctx.vsrc_mv = TKFX_ERROR_VALUE_ANALOG_16BITS;
			tkfx_ctx.vcap_mv = TKFX_ERROR_VALUE_ANALOG_12BITS;
			tkfx_ctx.vmcu_mv = TKFX_ERROR_VALUE_ANALOG_12BITS;
			tkfx_ctx.tmcu_degrees = TKFX_ERROR_VALUE_TEMPERATURE;
			if (adc1_status == ADC_SUCCESS) {
				// Read Vsrc.
				adc1_status = ADC1_get_data(ADC_DATA_INDEX_VSRC_MV, &generic_data_u32);
				ADC1_stack_error();
				if (adc1_status == ADC_SUCCESS) {
					tkfx_ctx.vsrc_mv = generic_data_u32;
				}
				// Read VCAP.
				adc1_status = ADC1_get_data(ADC_DATA_INDEX_VCAP_MV, &generic_data_u32);
				ADC1_stack_error();
				if (adc1_status == ADC_SUCCESS) {
					tkfx_ctx.vcap_mv = generic_data_u32;
				}
				// Read VMCU.
				adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &generic_data_u32);
				ADC1_stack_error();
				if (adc1_status == ADC_SUCCESS) {
					tkfx_ctx.vmcu_mv = generic_data_u32;
				}
				// Read TMCU.
				adc1_status = ADC1_get_tmcu(&temperature);
				ADC1_stack_error();
				if (adc1_status == ADC_SUCCESS) {
					math_status = MATH_int32_to_signed_magnitude((int32_t) temperature, 7, &generic_data_u32);
					MATH_stack_error();
					if (math_status == MATH_SUCCESS) {
						tkfx_ctx.tmcu_degrees = (uint8_t) generic_data_u32;
					}
				}
			}
			// Get GPS backup status.
			tkfx_ctx.status.gps_backup_status = NEOM8N_get_backup();
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_MODE_UPDATE;
			break;
		case TKFX_STATE_MODE_UPDATE:
#ifdef SSM
			// Check supercap voltage.
			tkfx_ctx.mode = (tkfx_ctx.vcap_mv < ((tkfx_ctx.config) -> vcap_min_mv)) ? TKFX_MODE_LOW_POWER : TKFX_MODE_ACTIVE;
#endif
#ifdef PM
			// Force mode to low power to disable accelerometer.
			tkfx_ctx.mode = TKFX_MODE_LOW_POWER;
#endif
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_ACCELERO;
			break;
		case TKFX_STATE_ACCELERO:
			IWDG_reload();
			// Configure accelerometer according to mode.
			if ((tkfx_ctx.mode == TKFX_MODE_ACTIVE) && ((tkfx_ctx.status.accelerometer_status == 0) || (tkfx_ctx.flags.por != 0))) {
				// Active mode.
				power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
				POWER_stack_error();
				mma8653fc_status = MMA8653FC_write_config(&(mma8653_active_config[0]), MMA8653FC_ACTIVE_CONFIG_LENGTH);
				MMA8653FC_stack_error();
				power_status = POWER_disable(POWER_DOMAIN_SENSORS);
				POWER_stack_error();
				// Update status.
				tkfx_ctx.status.accelerometer_status = 1;
			}
			if ((tkfx_ctx.mode == TKFX_MODE_LOW_POWER) && ((tkfx_ctx.status.accelerometer_status != 0) || (tkfx_ctx.flags.por != 0))) {
				// Sleep mode.
				power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
				POWER_stack_error();
				mma8653fc_status = MMA8653FC_write_config(&(mma8653_sleep_config[0]), MMA8653FC_SLEEP_CONFIG_LENGTH);
				MMA8653FC_stack_error();
				power_status = POWER_disable(POWER_DOMAIN_SENSORS);
				POWER_stack_error();
				// Update status.
				tkfx_ctx.status.accelerometer_status = 0;
			}
			// Compute next state.
			tkfx_ctx.state = (tkfx_ctx.flags.por != 0) ? TKFX_STATE_ERROR_STACK : TKFX_STATE_MONITORING;
			break;
		case TKFX_STATE_MONITORING:
			IWDG_reload();
			// Get clocks status.
			tkfx_ctx.status.lsi_status = RCC_get_lsi_status();
			tkfx_ctx.status.lse_status = RCC_get_lse_status();
			// Build Sigfox frame.
			tkfx_ctx.sigfox_monitoring_data.tamb_degrees = tkfx_ctx.tamb_degrees;
			tkfx_ctx.sigfox_monitoring_data.hamb_degrees = tkfx_ctx.hamb_percent;
			tkfx_ctx.sigfox_monitoring_data.tmcu_degrees = tkfx_ctx.tmcu_degrees;
			tkfx_ctx.sigfox_monitoring_data.vsrc_mv = tkfx_ctx.vsrc_mv;
			tkfx_ctx.sigfox_monitoring_data.vcap_mv = tkfx_ctx.vcap_mv;
			tkfx_ctx.sigfox_monitoring_data.vmcu_mv = tkfx_ctx.vmcu_mv;
			tkfx_ctx.sigfox_monitoring_data.status = tkfx_ctx.status.all;
			// Send uplink monitoring frame.
			application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
			application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_monitoring_data.frame);
			application_message.ul_payload_size_bytes = TKFX_SIGFOX_MONITORING_DATA_SIZE;
			_TKFX_send_sigfox_message(&application_message);
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_GEOLOC_CHECK;
			break;
		case TKFX_STATE_GEOLOC_CHECK:
#ifdef SSM
			// Go to off state by default.
			tkfx_ctx.state = TKFX_STATE_OFF;
			// Check mode.
			if (tkfx_ctx.mode == TKFX_MODE_LOW_POWER) {
				NEOM8N_set_backup(0);
			}
			else {
				// Check stop detection.
				if ((tkfx_ctx.status.moving_flag == 0) && (tkfx_ctx.status.alarm_flag != 0)) {
					tkfx_ctx.state = TKFX_STATE_GEOLOC;
				}
				else {
					// Check inactivity period.
					if ((((tkfx_ctx.config) -> inactivity_geoloc_enabled) != 0) && (tkfx_ctx.geoloc_timer_seconds >= ((tkfx_ctx.config) -> inactivity_threshold_seconds))) {
						tkfx_ctx.state = TKFX_STATE_GEOLOC;
					}
				}
			}
#endif
#ifdef PM
			// Check geoloc period.
			tkfx_ctx.state = (tkfx_ctx.geoloc_timer_seconds >= ((tkfx_ctx.config) -> geoloc_period_seconds)) ? TKFX_STATE_GEOLOC : TKFX_STATE_OFF;
#endif
			break;
		case TKFX_STATE_GEOLOC:
			IWDG_reload();
			// Enable backup.
			NEOM8N_set_backup(1);
			// Get position from GPS.
			power_status = POWER_enable(POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_STOP);
			POWER_stack_error();
			neom8n_status = NEOM8N_get_position(&tkfx_ctx.geoloc_position, ((tkfx_ctx.config) -> geoloc_timeout_seconds), ((tkfx_ctx.config) -> vcap_min_mv), &tkfx_ctx.geoloc_fix_duration_seconds);
			// Note: error is never stacked since it is indicated by the dedicated timeout frame.
			power_status = POWER_disable(POWER_DOMAIN_GPS);
			POWER_stack_error();
			// Build Sigfox frame.
			if (neom8n_status == NEOM8N_SUCCESS) {
				tkfx_ctx.sigfox_geoloc_data.latitude_degrees = tkfx_ctx.geoloc_position.lat_degrees;
				tkfx_ctx.sigfox_geoloc_data.latitude_minutes = tkfx_ctx.geoloc_position.lat_minutes;
				tkfx_ctx.sigfox_geoloc_data.latitude_seconds = tkfx_ctx.geoloc_position.lat_seconds;
				tkfx_ctx.sigfox_geoloc_data.latitude_north_flag = tkfx_ctx.geoloc_position.lat_north_flag;
				tkfx_ctx.sigfox_geoloc_data.longitude_degrees = tkfx_ctx.geoloc_position.long_degrees;
				tkfx_ctx.sigfox_geoloc_data.longitude_minutes = tkfx_ctx.geoloc_position.long_minutes;
				tkfx_ctx.sigfox_geoloc_data.longitude_seconds = tkfx_ctx.geoloc_position.long_seconds;
				tkfx_ctx.sigfox_geoloc_data.longitude_east_flag = tkfx_ctx.geoloc_position.long_east_flag;
				tkfx_ctx.sigfox_geoloc_data.altitude_meters = tkfx_ctx.geoloc_position.altitude;
				tkfx_ctx.sigfox_geoloc_data.gps_fix_duration_seconds = tkfx_ctx.geoloc_fix_duration_seconds;
				// Update message parameters.
				application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
				application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_geoloc_data.frame);
				application_message.ul_payload_size_bytes = TKFX_SIGFOX_GEOLOC_DATA_SIZE;
			}
			else {
				tkfx_ctx.sigfox_geoloc_timeout_data.error_code = (ERROR_BASE_NEOM8N + neom8n_status);
				tkfx_ctx.sigfox_geoloc_timeout_data.fix_duration_seconds = tkfx_ctx.geoloc_fix_duration_seconds;
				// Update message parameters.
				application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
				application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_geoloc_timeout_data.frame);
				application_message.ul_payload_size_bytes = TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_SIZE;
			}
			// Send uplink geolocation frame.
			_TKFX_send_sigfox_message(&application_message);
			// Reset geoloc variables.
			tkfx_ctx.geoloc_fix_duration_seconds = 0;
			tkfx_ctx.geoloc_timer_seconds = 0,
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_ERROR_STACK;
			break;
		case TKFX_STATE_ERROR_STACK:
			// Import Sigfox library error stack.
			ERROR_import_sigfox_stack();
			// Check stack.
			if (ERROR_stack_is_empty() == 0) {
				// Read error stack.
				for (idx=0 ; idx<(TKFX_SIGFOX_ERROR_STACK_DATA_SIZE / 2) ; idx++) {
					error_code = ERROR_stack_read();
					tkfx_ctx.sigfox_error_stack_data[(2 * idx) + 0] = (uint8_t) ((error_code >> 8) & 0x00FF);
					tkfx_ctx.sigfox_error_stack_data[(2 * idx) + 1] = (uint8_t) ((error_code >> 0) & 0x00FF);
				}
				// Send error stack frame.
				application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
				application_message.ul_payload = (sfx_u8*) (tkfx_ctx.sigfox_error_stack_data);
				application_message.ul_payload_size_bytes = TKFX_SIGFOX_ERROR_STACK_DATA_SIZE;
				_TKFX_send_sigfox_message(&application_message);
			}
			// Enter sleep mode.
			tkfx_ctx.state = TKFX_STATE_OFF;
			break;
		case TKFX_STATE_OFF:
			IWDG_reload();
			// Clear POR flag.
			tkfx_ctx.flags.por = 0;
#ifdef SSM
			// Reset algorithm variables.
			tkfx_ctx.start_detection_irq_count = 0;
			tkfx_ctx.stop_detection_timer_seconds = 0;
			tkfx_ctx.keep_alive_timer_seconds = 0;
#endif
			// Clear RTC flag.
			RTC_clear_wakeup_timer_flag();
#ifdef SSM
			// Enable accelerometer interrupt if required.
			if (tkfx_ctx.status.accelerometer_status != 0) {
				// Enable interrupt.
				MMA8653FC_clear_motion_interrupt_flag();
				MMA8653_enable_motion_interrupt();
			}
#endif
			// Enter sleep mode.
			tkfx_ctx.state = TKFX_STATE_SLEEP;
			break;
		case TKFX_STATE_SLEEP:
			// Enter stop mode.
			IWDG_reload();
			PWR_enter_stop_mode();
			IWDG_reload();
			// Check wake-up source.
			if (RTC_get_wakeup_timer_flag() != 0) {
				// Increment timers.
				tkfx_ctx.geoloc_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
#ifdef SSM
				// Decrement IRQ count.
				if (tkfx_ctx.start_detection_irq_count > 0) {
					tkfx_ctx.start_detection_irq_count--;
				}
				// Increment timers.
				tkfx_ctx.keep_alive_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
				tkfx_ctx.stop_detection_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
				// Check keep-alive period.
				if (tkfx_ctx.keep_alive_timer_seconds >= ((tkfx_ctx.config) -> keep_alive_period_seconds)) {
					// Reset alarm flag.
					tkfx_ctx.status.alarm_flag = 0;
					// Turn tracker on to send keep-alive.
					tkfx_ctx.state = TKFX_STATE_WAKEUP;
				}
#endif
#ifdef PM
				// Check geoloc period.
				if (tkfx_ctx.geoloc_timer_seconds >= ((tkfx_ctx.config) -> geoloc_period_seconds)) {
					// Reset alarm flag.
					tkfx_ctx.status.alarm_flag = 0;
					// Turn tracker on to send keep-alive.
					tkfx_ctx.state = TKFX_STATE_WAKEUP;
				}
#endif
				// Clear RTC flags.
				RTC_clear_wakeup_timer_flag();
			}
#ifdef SSM
			if (MMA8653FC_get_motion_interrupt_flag() != 0) {
				// Increment IRQ count and reset stop timer.
				tkfx_ctx.start_detection_irq_count++;
				tkfx_ctx.stop_detection_timer_seconds = 0;
				// Wake-up from accelerometer interrupt.
				if ((tkfx_ctx.status.moving_flag == 0) && ((tkfx_ctx.start_detection_irq_count) > ((tkfx_ctx.config) -> start_detection_threshold_irq))) {
					// Start condition detected.
					tkfx_ctx.status.moving_flag = 1;
					tkfx_ctx.status.alarm_flag = 1;
					// Turn tracker on to send start alarm.
					tkfx_ctx.state = TKFX_STATE_WAKEUP;
				}
				MMA8653FC_clear_motion_interrupt_flag();
			}
			else {
				// No movement detected.
				if ((tkfx_ctx.stop_detection_timer_seconds >= ((tkfx_ctx.config) -> stop_detection_threshold_seconds)) && (tkfx_ctx.status.moving_flag != 0)) {
					// Stop condition detected.
					tkfx_ctx.status.moving_flag = 0;
					tkfx_ctx.status.alarm_flag = 1;
					// Turn tracker on to send stop alarm.
					tkfx_ctx.state = TKFX_STATE_WAKEUP;
				}
			}
#endif
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

#ifdef ATM
/*******************************************************************/
int main (void) {
	// Init board.
	_TKFX_init_hw();
	// Local variables.
	POWER_status_t power_status = POWER_SUCCESS;
	MMA8653FC_status_t mma8653fc_status = MMA8653FC_SUCCESS;
	// Configure accelerometer.
	power_status = POWER_enable(POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
	POWER_stack_error();
	mma8653fc_status = MMA8653FC_write_config(&(mma8653_active_config[0]), MMA8653FC_ACTIVE_CONFIG_LENGTH);
	MMA8653FC_stack_error();
	power_status = POWER_disable(POWER_DOMAIN_SENSORS);
	POWER_stack_error();
	// Main loop.
	while (1) {
		// Enter sleep mode.
		IWDG_reload();
		PWR_enter_sleep_mode();
		IWDG_reload();
		// Perform AT commands task.
		AT_task();
	}
	return 0;
}
#endif

