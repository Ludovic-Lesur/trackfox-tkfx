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
#include "aes.h"
#include "exti.h"
#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "mapping.h"
#include "nvic.h"
#include "nvm.h"
#include "pwr.h"
#include "rcc.h"
#include "spi.h"
#include "rtc.h"
#include "usart.h"
// Components.
#include "mma8653fc.h"
#include "neom8n.h"
#include "s2lp.h"
#include "sht3x.h"
#include "sigfox_types.h"
// Applicative.
#include "at.h"
#include "mode.h"
#include "sigfox_api.h"

/*** MAIN macros ***/

#define TKFX_SIGFOX_GEOLOC_DATA_LENGTH_BYTES			11
#define TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH_BYTES	1
#define TKFX_SIGFOX_DOWNLINK_DATA_LENGTH_BYTES			8
#define TKFX_SIGFOX_MONITORING_DATA_LENGTH_BYTES		8

/*** MAIN structures ***/

#ifndef ATM

// State machine.
typedef enum {
	TKFX_STATE_INIT,
	TKFX_STATE_OOB,
	TKFX_STATE_MEASURE,
	TKFX_STATE_MONITORING,
	TKFX_STATE_VCAP_CHECK,
	TKFX_GEOLOC_CHECK,
	TKFX_STATE_GEOLOC,
#ifdef SSM
	TKFX_STATE_ACCELERO,
#endif
	TKFX_STATE_OFF,
	TKFX_STATE_SLEEP,
	TKFX_STATE_LAST
} TKFX_state_t;

typedef union {
	unsigned char raw_byte;
	struct {
		unsigned gps_backup_status : 1;
		unsigned accelerometer_status : 1;
		unsigned lse_status : 1;
		unsigned lsi_status : 1;
		unsigned moving_flag : 1;
		unsigned alarm_flag : 1;
		unsigned tracker_mode : 2;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} TKFX_status_t;

// Sigfox monitoring frame data format.
typedef union {
	unsigned char raw_frame[TKFX_SIGFOX_MONITORING_DATA_LENGTH_BYTES];
	struct {
		unsigned tamb_degrees : 8;
		unsigned tmcu_degrees : 8;
		unsigned vsrc_mv : 16;
		unsigned vcap_mv : 12;
		unsigned vmcu_mv : 12;
		unsigned status : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} TKFX_sigfox_monitoring_data_t;

// Sigfox geoloc frame data format.
typedef union {
	unsigned char raw_frame[TKFX_SIGFOX_GEOLOC_DATA_LENGTH_BYTES];
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
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} TKFX_sigfox_geoloc_data_t;

typedef struct {
	unsigned int vcap_min_mv;
	unsigned int geoloc_timeout_seconds;
#ifdef SSM
	unsigned int start_detection_threshold_irq; // Number of accelerometer interrupts required to trigger start conidition (set to 0 to disable filter).
	unsigned int stop_detection_threshold_seconds; // Delay required to trigger stop condition.
	unsigned int keep_alive_period_seconds;
	unsigned int inactivity_geoloc_enabled; // If non zero, force a GPS fix after an inactivity period.
	unsigned int inactivity_threshold_seconds;
#endif
#ifdef PM
	unsigned int geoloc_period_seconds;
#endif
} TKFX_config_t;

// Device context.
typedef struct {
	// State machine.
	TKFX_state_t state;
	unsigned char por_flag;
	unsigned int lsi_frequency_hz;
	const TKFX_config_t* config;
#ifdef SSM
	unsigned int start_detection_irq_count;
	unsigned int stop_detection_timer_seconds;
	unsigned int keep_alive_timer_seconds;
#endif
	unsigned int geoloc_timer_seconds;
	TKFX_status_t status;
	// Monitoring data.
	unsigned char tamb_degrees;
	unsigned char tmcu_degrees;
	unsigned int vsrc_mv;
	unsigned int vcap_mv;
	unsigned int vmcu_mv;
	// Geoloc.
	NEOM8N_position_t geoloc_position;
	unsigned int geoloc_fix_duration_seconds;
	unsigned char geoloc_timeout_flag;
	// Sigfox.
	TKFX_sigfox_monitoring_data_t sigfox_monitoring_data;
	TKFX_sigfox_geoloc_data_t sigfox_geoloc_data;
	unsigned char sigfox_downlink_data[TKFX_SIGFOX_DOWNLINK_DATA_LENGTH_BYTES];
} TKFX_context_t;

/*** MAIN global variables ***/

static TKFX_context_t tkfx_ctx;
#ifdef SSM
//static const TKFX_config_t tkfx_config = {1500, 180, 0, 300, 3600, 1, 86400}; // Car tracking configuration.
static const TKFX_config_t tkfx_config = {1500, 180, 5, 60, 3600, 0, 86400}; // Hiking configuration.
#endif
#ifdef PM
static const TKFX_config_t tkfx_config = {1500, 180, 300}; // Bike tracking configuration.
#endif

/*** MAIN functions ***/

/* MAIN FUNCTION FOR START/STOP MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init memory.
	NVIC_init();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
	// Init clock and power modules.
	RCC_init();
	PWR_init();
	// Reset RTC before starting oscillators.
	RTC_reset();
	// Start LSI.
	unsigned char lsi_success = RCC_enable_lsi();
	// Init watchdog.
#ifndef DEBUG
	IWDG_init();
#endif
	IWDG_reload();
	// Start LSE.
	unsigned char lse_success = RCC_enable_lse();
	// Switch to HSI clock.
	RCC_switch_to_hsi();
	// Get LSI effective frequency (must be called after HSI initialization and before RTC inititialization).
	RCC_get_lsi_frequency(&tkfx_ctx.lsi_frequency_hz);
	IWDG_reload();
	// Init RTC.
	RTC_init(&lse_success, tkfx_ctx.lsi_frequency_hz);
	// Local variables.
	sfx_error_t sfx_error = 0;
	sfx_rc_t tkfx_sigfox_rc = (sfx_rc_t) RC1;
	NEOM8N_status_t neom8n_status = NEOM8N_ERROR_TIMEOUT;
	// Init context.
	tkfx_ctx.state = TKFX_STATE_INIT;
	tkfx_ctx.por_flag = 1;
	tkfx_ctx.config = &tkfx_config;
	tkfx_ctx.geoloc_timer_seconds = 0;
	tkfx_ctx.status.raw_byte = 0x00; // Reset all status flags.
	tkfx_ctx.status.field.lsi_status = lsi_success;
	tkfx_ctx.status.field.lse_status = lse_success;
#ifdef SSM
	tkfx_ctx.status.field.tracker_mode = 0b00;
#endif
#ifdef PM
	tkfx_ctx.status.field.tracker_mode = 0b01;
#endif
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (tkfx_ctx.state) {
		case TKFX_STATE_INIT:
			IWDG_reload();
			// Disable RTC interrupt.
			RTC_stop_wakeup_timer();
			// Init context.
#ifdef SSM
			tkfx_ctx.start_detection_irq_count = 0;
			tkfx_ctx.stop_detection_timer_seconds = 0;
			tkfx_ctx.keep_alive_timer_seconds = 0;
#endif
			// High speed oscillator.
			IWDG_reload();
			RCC_switch_to_hsi();
			// Init peripherals.
			LPTIM1_init(tkfx_ctx.lsi_frequency_hz);
			ADC1_init();
			USART2_init();
			I2C1_init();
			LPUART1_init(lse_success);
			// Init components.
			NEOM8N_init();
			SHT3X_init();
#ifdef SSM
			MMA8653FC_init();
			// Disable accelerometer interrupt.
			MMA8653FC_clear_motion_interrupt_flag();
			NVIC_disable_interrupt(NVIC_IT_EXTI_0_1);
			// Turn accelerometer off.
			I2C1_power_on();
			MMA8653FC_write_config(&(mma8653_sleep_config[0]), MMA8653FC_SLEEP_CONFIG_LENGTH);
			I2C1_power_off();
#endif
			// Compute next state.
			tkfx_ctx.state = (tkfx_ctx.por_flag != 0) ? TKFX_STATE_OOB : TKFX_STATE_MEASURE;
			break;
		case TKFX_STATE_OOB:
			IWDG_reload();
			// Send OOB frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
			}
			SIGFOX_API_close();
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_MEASURE;
			break;
		case TKFX_STATE_MEASURE:
			IWDG_reload();
			// Get temperature from SHT30.
			I2C1_power_on();
			SHT3X_perform_measurements();
			I2C1_power_off();
			SHT3X_get_temperature_comp1(&tkfx_ctx.tamb_degrees);
			// Get voltages measurements.
			ADC1_power_on();
			ADC1_perform_measurements();
			ADC1_power_off();
			ADC1_get_data(ADC_DATA_IDX_VSRC_MV, &tkfx_ctx.vsrc_mv);
			ADC1_get_data(ADC_DATA_IDX_VCAP_MV, &tkfx_ctx.vcap_mv);
			ADC1_get_data(ADC_DATA_IDX_VMCU_MV, &tkfx_ctx.vmcu_mv);
			ADC1_get_tmcu_comp1(&tkfx_ctx.tmcu_degrees);
			// Get GPS backup status.
			tkfx_ctx.status.field.gps_backup_status = NEOM8N_get_backup();
			// Compute next state.
			tkfx_ctx.state = (tkfx_ctx.por_flag != 0) ? TKFX_STATE_VCAP_CHECK : TKFX_STATE_MONITORING;
			break;
		case TKFX_STATE_MONITORING:
			IWDG_reload();
			// Build Sigfox frame.
			tkfx_ctx.sigfox_monitoring_data.field.tamb_degrees = tkfx_ctx.tamb_degrees;
			tkfx_ctx.sigfox_monitoring_data.field.tmcu_degrees = tkfx_ctx.tmcu_degrees;
			tkfx_ctx.sigfox_monitoring_data.field.vsrc_mv = tkfx_ctx.vsrc_mv;
			tkfx_ctx.sigfox_monitoring_data.field.vcap_mv = tkfx_ctx.vcap_mv;
			tkfx_ctx.sigfox_monitoring_data.field.vmcu_mv = tkfx_ctx.vmcu_mv;
			tkfx_ctx.sigfox_monitoring_data.field.status = tkfx_ctx.status.raw_byte;
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(tkfx_ctx.sigfox_monitoring_data.raw_frame, TKFX_SIGFOX_MONITORING_DATA_LENGTH_BYTES, tkfx_ctx.sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Reset accelerometer flag (must be done after monitoring frame sending).
			tkfx_ctx.status.field.accelerometer_status = 0;
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_VCAP_CHECK;
			break;
		case TKFX_STATE_VCAP_CHECK:
			// Check supercap voltage.
			if (tkfx_ctx.vcap_mv < ((tkfx_ctx.config) -> vcap_min_mv)) {
				// Low power mode: disable GPS backup, do not initialize accelerometer and never perform GPS fix.
				NEOM8N_set_backup(0);
				tkfx_ctx.state = TKFX_STATE_OFF;
			}
			else {
				tkfx_ctx.state = TKFX_GEOLOC_CHECK;
			}
			break;
		case TKFX_GEOLOC_CHECK:
#ifdef SSM
			// Check stop detection.
			if ((tkfx_ctx.status.field.moving_flag == 0) && (tkfx_ctx.status.field.alarm_flag != 0)) {
				tkfx_ctx.state = TKFX_STATE_GEOLOC;
			}
			else {
				// Check inactivity period.
				if ((((tkfx_ctx.config) -> inactivity_geoloc_enabled) != 0) && (tkfx_ctx.geoloc_timer_seconds >= ((tkfx_ctx.config) -> inactivity_threshold_seconds))) {
					tkfx_ctx.state = TKFX_STATE_GEOLOC;
				}
				else {
					// Nominal mode but no geoloc required: configure accelerometer.
					tkfx_ctx.state = TKFX_STATE_ACCELERO;
				}
			}
#endif
#ifdef PM
			// Check geoloc period.
			if (tkfx_ctx.geoloc_timer_seconds >= ((tkfx_ctx.config) -> geoloc_period_seconds)) {
				tkfx_ctx.state = TKFX_STATE_GEOLOC;
			}
			else {
				tkfx_ctx.state = TKFX_STATE_OFF;
			}
#endif
			break;
		case TKFX_STATE_GEOLOC:
			IWDG_reload();
			// Enable backup.
			NEOM8N_set_backup(1);
			// Get position from GPS.
			LPUART1_power_on();
			neom8n_status = NEOM8N_get_position(&tkfx_ctx.geoloc_position, ((tkfx_ctx.config) -> geoloc_timeout_seconds), ((tkfx_ctx.config) -> vcap_min_mv), &tkfx_ctx.geoloc_fix_duration_seconds);
			LPUART1_power_off();
			// Parse result.
			tkfx_ctx.geoloc_timeout_flag = (neom8n_status == NEOM8N_SUCCESS) ? 0 : 1;
			// Build Sigfox frame.
			if (tkfx_ctx.geoloc_timeout_flag == 0) {
				tkfx_ctx.sigfox_geoloc_data.field.latitude_degrees = tkfx_ctx.geoloc_position.lat_degrees;
				tkfx_ctx.sigfox_geoloc_data.field.latitude_minutes = tkfx_ctx.geoloc_position.lat_minutes;
				tkfx_ctx.sigfox_geoloc_data.field.latitude_seconds = tkfx_ctx.geoloc_position.lat_seconds;
				tkfx_ctx.sigfox_geoloc_data.field.latitude_north_flag = tkfx_ctx.geoloc_position.lat_north_flag;
				tkfx_ctx.sigfox_geoloc_data.field.longitude_degrees = tkfx_ctx.geoloc_position.long_degrees;
				tkfx_ctx.sigfox_geoloc_data.field.longitude_minutes = tkfx_ctx.geoloc_position.long_minutes;
				tkfx_ctx.sigfox_geoloc_data.field.longitude_seconds = tkfx_ctx.geoloc_position.long_seconds;
				tkfx_ctx.sigfox_geoloc_data.field.longitude_east_flag = tkfx_ctx.geoloc_position.long_east_flag;
				tkfx_ctx.sigfox_geoloc_data.field.altitude_meters = tkfx_ctx.geoloc_position.altitude;
				tkfx_ctx.sigfox_geoloc_data.field.gps_fix_duration_seconds = tkfx_ctx.geoloc_fix_duration_seconds;
			}
			else {
				tkfx_ctx.sigfox_geoloc_data.raw_frame[0] = tkfx_ctx.geoloc_fix_duration_seconds;
			}
			// Send uplink geolocation frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(tkfx_ctx.sigfox_geoloc_data.raw_frame, (tkfx_ctx.geoloc_timeout_flag ? TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH_BYTES : TKFX_SIGFOX_GEOLOC_DATA_LENGTH_BYTES), tkfx_ctx.sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Reset geoloc variables.
			tkfx_ctx.geoloc_timeout_flag = 0;
			tkfx_ctx.geoloc_fix_duration_seconds = 0;
			tkfx_ctx.geoloc_timer_seconds = 0,
			// Compute next state.
#ifdef SSM
			tkfx_ctx.state = TKFX_STATE_ACCELERO;
#else
			tkfx_ctx.state = TKFX_STATE_OFF;
#endif
			break;
#ifdef SSM
		case TKFX_STATE_ACCELERO:
			IWDG_reload();
			// Configure accelerometer for motion detection.
			I2C1_power_on();
			MMA8653FC_write_config(&(mma8653_active_config[0]), MMA8653FC_ACTIVE_CONFIG_LENGTH);
			I2C1_power_off();
			// Enable interrupt.
			tkfx_ctx.start_detection_irq_count = 0;
			MMA8653FC_clear_motion_interrupt_flag();
			NVIC_enable_interrupt(NVIC_IT_EXTI_0_1);
			// Set status flag.
			tkfx_ctx.status.field.accelerometer_status = 1;
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_OFF;
			break;
#endif
		case TKFX_STATE_OFF:
			IWDG_reload();
			// Clear POR flag.
			tkfx_ctx.por_flag = 0;
			// Turn peripherals off.
			ADC1_disable();
			LPTIM1_disable();
			I2C1_disable();
			LPUART1_disable();
			// Clear EXTI flags.
			EXTI_clear_all_flags();
			// Enable RTC interrupt.
			RTC_clear_wakeup_timer_flag();
			RTC_start_wakeup_timer(RTC_WAKEUP_PERIOD_SECONDS);
			// Enter sleep mode.
			tkfx_ctx.state = TKFX_STATE_SLEEP;
			break;
		case TKFX_STATE_SLEEP:
			IWDG_reload();
			// Enter stop mode.
			PWR_enter_stop_mode();
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
					tkfx_ctx.status.field.alarm_flag = 0;
					// Turn tracker on to send keep-alive.
					tkfx_ctx.state = TKFX_STATE_INIT;
				}
#endif
#ifdef PM
				// Check geoloc period.
				if (tkfx_ctx.geoloc_timer_seconds >= ((tkfx_ctx.config) -> geoloc_period_seconds)) {
					// Reset alarm flag.
					tkfx_ctx.status.field.alarm_flag = 0;
					// Turn tracker on to send keep-alive.
					tkfx_ctx.state = TKFX_STATE_INIT;
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
				if ((tkfx_ctx.status.field.moving_flag == 0) && ((tkfx_ctx.start_detection_irq_count) > ((tkfx_ctx.config) -> start_detection_threshold_irq))) {
					// Start condition detected.
					tkfx_ctx.status.field.moving_flag = 1;
					tkfx_ctx.status.field.alarm_flag = 1;
					// Turn tracker on to send start alarm.
					tkfx_ctx.state = TKFX_STATE_INIT;
				}
				MMA8653FC_clear_motion_interrupt_flag();
			}
			else {
				// No movement detected.
				if ((tkfx_ctx.stop_detection_timer_seconds >= ((tkfx_ctx.config) -> stop_detection_threshold_seconds)) && (tkfx_ctx.status.field.moving_flag != 0)) {
					// Stop condition detected.
					tkfx_ctx.status.field.moving_flag = 0;
					tkfx_ctx.status.field.alarm_flag = 1;
					// Turn tracker on to send stop alarm.
					tkfx_ctx.state = TKFX_STATE_INIT;
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
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Start LSI clock and watchdog.
	RCC_enable_lsi();
#ifndef DEBUG
	IWDG_init();
#endif
	// Init memory.
	NVIC_init();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
	// Reset RTC before starting oscillators.
	RTC_reset();
	// Init clocks.
	RCC_init();
	unsigned char tkfx_use_lse = RCC_enable_lse();
	RCC_switch_to_hsi();
	// Get LSI effective frequency (must be called after HSI initialization and before RTC inititialization).
	unsigned int lsi_frequency_hz = 0;
	RCC_get_lsi_frequency(&lsi_frequency_hz);
	IWDG_reload();
	// Init RTC and timers.
	RTC_init(&tkfx_use_lse, lsi_frequency_hz);
	LPTIM1_init(lsi_frequency_hz);
	// Init peripherals.
	ADC1_init();
	USART2_init();
	I2C1_init();
	LPUART1_init(tkfx_use_lse);
	// Components.
	NEOM8N_init();
	SHT3X_init();
	MMA8653FC_init();
	// Configure accelerometer.
	I2C1_power_on();
	MMA8653FC_write_config(&(mma8653_active_config[0]), MMA8653FC_ACTIVE_CONFIG_LENGTH);
	I2C1_power_off();
	// Applicative layers.
	AT_init();
	// Main loop.
	while (1) {
		AT_task();
		IWDG_reload();
	}
	return 0;
}
#endif

