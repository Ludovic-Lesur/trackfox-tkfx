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
#include "dma.h"
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
#include "error.h"
#include "mode.h"
#include "sigfox_api.h"
#include "version.h"

/*** MAIN macros ***/

// Sigfox payload lengths.
#define TKFX_SIGFOX_STARTUP_DATA_LENGTH			8
#define TKFX_SIGFOX_GEOLOC_DATA_LENGTH			11
#define TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH	1
#define TKFX_SIGFOX_DOWNLINK_DATA_LENGTH		8
#define TKFX_SIGFOX_MONITORING_DATA_LENGTH		9
#define TKFX_SIGFOX_ERROR_STACK_DATA_LENGTH	(ERROR_STACK_DEPTH * 2)
// Error values.
#define TKFX_ERROR_VALUE_VOLTAGE_12BITS			0xFFF
#define TKFX_ERROR_VALUE_VOLTAGE_16BITS			0xFFFF
#define TKFX_ERROR_VALUE_TEMPERATURE			0x7F
#define TKFX_ERROR_VALUE_HUMIDITY				0xFF

/*** MAIN structures ***/

// State machine.
typedef enum {
	TKFX_STATE_STARTUP,
	TKFX_STATE_WAKEUP,
	TKFX_STATE_MEASURE,
	TKFX_STATE_MONITORING,
	TKFX_STATE_MODE_UPDATE,
	TKFX_GEOLOC_CHECK,
	TKFX_STATE_GEOLOC,
#ifdef SSM
	TKFX_STATE_ACCELERO,
#endif
	TKFX_STATE_OFF,
	TKFX_STATE_SLEEP,
	TKFX_STATE_LAST
} TKFX_state_t;

typedef enum {
	TKFX_MODE_ACTIVE,
	TKFX_MODE_LOW_POWER,
	TKFX_MODE_LAST
} TKFX_mode_t;

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
	unsigned char all;
} TKFX_status_t;

typedef union {
	struct {
		unsigned por : 1;
		unsigned geoloc_timeout : 1;
	};
	unsigned char all;
} TKFX_flags_t;

// Sigfox start-up frame data format.
typedef union {
	unsigned char frame[TKFX_SIGFOX_STARTUP_DATA_LENGTH];
	struct {
		unsigned reset_reason : 8;
		unsigned major_version : 8;
		unsigned minor_version : 8;
		unsigned commit_index : 8;
		unsigned commit_id : 28;
		unsigned dirty_flag : 4;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} TKFX_sigfox_startup_data_t;

// Sigfox monitoring frame data format.
typedef union {
	unsigned char frame[TKFX_SIGFOX_MONITORING_DATA_LENGTH];
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

// Sigfox geoloc frame data format.
typedef union {
	unsigned char frame[TKFX_SIGFOX_GEOLOC_DATA_LENGTH];
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
	// Global.
	TKFX_state_t state;
	TKFX_mode_t mode;
	TKFX_flags_t flags;
	const TKFX_config_t* config;
	// Clocks.
	unsigned int lsi_frequency_hz;
	unsigned char lse_running;
	// Wake-up management.
#ifdef SSM
	unsigned int start_detection_irq_count;
	unsigned int stop_detection_timer_seconds;
	unsigned int keep_alive_timer_seconds;
#endif
	unsigned int geoloc_timer_seconds;
	// SW version.
	TKFX_sigfox_startup_data_t sigfox_startup_data;
	// Monitoring.
	TKFX_status_t status;
	unsigned char tamb_degrees;
	unsigned char tmcu_degrees;
	unsigned int vsrc_mv;
	unsigned int vcap_mv;
	unsigned int vmcu_mv;
	TKFX_sigfox_monitoring_data_t sigfox_monitoring_data;
	// Geoloc.
	NEOM8N_position_t geoloc_position;
	unsigned int geoloc_fix_duration_seconds;
	TKFX_sigfox_geoloc_data_t sigfox_geoloc_data;
	// Error stack.
	ERROR_t error_stack[ERROR_STACK_DEPTH];
	unsigned char sigfox_error_stack_data[TKFX_SIGFOX_ERROR_STACK_DATA_LENGTH];
	// Sigfox.
	sfx_rc_t sigfox_rc;
	unsigned char sigfox_downlink_data[TKFX_SIGFOX_DOWNLINK_DATA_LENGTH];
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

/* COMMON INIT FUNCTION FOR MAIN CONTEXT.
 * @param:	None.
 * @return:	None.
 */
static void TKFX_init_context(void) {
	// Init context.
	tkfx_ctx.state = TKFX_STATE_STARTUP;
	tkfx_ctx.mode = TKFX_MODE_ACTIVE;
	tkfx_ctx.flags.all = 0;
	tkfx_ctx.flags.por = 1;
	tkfx_ctx.config = &tkfx_config;
	tkfx_ctx.lsi_frequency_hz = 0;
	tkfx_ctx.lse_running = 0;
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
	// Set Sigfox RC.
	tkfx_ctx.sigfox_rc = (sfx_rc_t) RC1;
}

/* COMMON INIT FUNCTION FOR PERIPHERALS AND COMPONENTS.
 * @param:	None.
 * @return:	None.
 */
static void TKFX_init_hw(void) {
	// Local variables.
	RCC_status_t rcc_status = RCC_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	ADC_status_t adc1_status = ADC_SUCCESS;
#ifndef DEBUG
	IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
	// Init error stack
	ERROR_stack_init();
	// Init memory.
	NVIC_init();
	NVM_init();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
	// Init clock and power modules.
	RCC_init();
	PWR_init();
	// Reset RTC.
	RTC_reset();
	// Start oscillators.
	rcc_status = RCC_enable_lsi();
	RCC_error_check();
	tkfx_ctx.status.lsi_status = (rcc_status == RCC_SUCCESS) ? 1 : 0;
	rcc_status = RCC_enable_lse();
	RCC_error_check();
	tkfx_ctx.status.lse_status = (rcc_status == RCC_SUCCESS) ? 1 : 0;
	// Start independent watchdog.
#ifndef DEBUG
	iwdg_status = IWDG_init();
	IWDG_error_check();
#endif
	// High speed oscillator.
	IWDG_reload();
	rcc_status = RCC_switch_to_hsi();
	RCC_error_check();
	// Get LSI effective frequency (must be called after HSI initialization and before RTC inititialization).
	rcc_status = RCC_get_lsi_frequency(&tkfx_ctx.lsi_frequency_hz);
	RCC_error_check();
	if (rcc_status != RCC_SUCCESS) tkfx_ctx.lsi_frequency_hz = RCC_LSI_FREQUENCY_HZ;
	IWDG_reload();
	// RTC.
	tkfx_ctx.lse_running = tkfx_ctx.status.lse_status;
	rtc_status = RTC_init(&tkfx_ctx.lse_running, tkfx_ctx.lsi_frequency_hz);
	RTC_error_check();
	// Update LSE status if RTC failed to start on it.
	if (tkfx_ctx.lse_running == 0) {
		tkfx_ctx.status.lse_status = 0;
	}
	IWDG_reload();
	// Internal.
	AES_init();
	DMA1_init_channel3();
	LPTIM1_init(tkfx_ctx.lsi_frequency_hz);
	// Analog.
	adc1_status = ADC1_init();
	ADC1_error_check();
	// Communication interfaces.
	LPUART1_init(tkfx_ctx.lse_running);
	I2C1_init();
	SPI1_init();
	USART2_init();
	// Init components.
	S2LP_init();
	NEOM8N_init();
#ifdef SSM
	MMA8653FC_init();
#endif
}

#if (defined SSM) || (defined PM)
/* MAIN FUNCTION FOR START/STOP MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init board.
	TKFX_init_context();
	TKFX_init_hw();
	// Local variables.
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	signed char temperature_degrees = 0;
	unsigned int generic_data_u32 = 0;
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
			// Clear reset flags.
			RCC -> CSR |= (0b1 << 23);
			// Send SW version frame.
			sigfox_api_status = SIGFOX_API_open(&tkfx_ctx.sigfox_rc);
			SIGFOX_API_error_check();
			if (sigfox_api_status == SFX_ERR_NONE) {
				sigfox_api_status = SIGFOX_API_send_frame(tkfx_ctx.sigfox_startup_data.frame, TKFX_SIGFOX_STARTUP_DATA_LENGTH, tkfx_ctx.sigfox_downlink_data, 2, 0);
				SIGFOX_API_error_check();
			}
			sigfox_api_status = SIGFOX_API_close();
			SIGFOX_API_error_check();
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_MEASURE;
			break;
		case TKFX_STATE_WAKEUP:
			IWDG_reload();
			// Disable RTC interrupt.
			RTC_stop_wakeup_timer();
#ifdef SSM
			// Disable accelerometer interrupt.
			MMA8653FC_clear_motion_interrupt_flag();
			NVIC_disable_interrupt(NVIC_IT_EXTI_0_1);
#endif
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_MEASURE;
			break;
		case TKFX_STATE_MEASURE:
			IWDG_reload();
			// Get temperature from SHT30.
			I2C1_power_on();
			SHT3X_perform_measurements(SHT3X_I2C_ADDRESS);
			I2C1_power_off();
			SHT3X_get_temperature(&temperature_degrees);
			MATH_one_complement(temperature_degrees, 7, &generic_data_u32);
			tkfx_ctx.tamb_degrees = (unsigned char) generic_data_u32;
			// Get voltages measurements.
			ADC1_power_on();
			ADC1_perform_measurements();
			ADC1_power_off();
			ADC1_get_data(ADC_DATA_INDEX_VSRC_MV, &tkfx_ctx.vsrc_mv);
			ADC1_get_data(ADC_DATA_INDEX_VCAP_MV, &tkfx_ctx.vcap_mv);
			ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &tkfx_ctx.vmcu_mv);
			ADC1_get_tmcu(&temperature_degrees);
			MATH_one_complement(temperature_degrees, 7, &generic_data_u32);
			tkfx_ctx.tmcu_degrees = (unsigned char) generic_data_u32;
			// Get GPS backup status.
			tkfx_ctx.status.gps_backup_status = NEOM8N_get_backup();
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_MODE_UPDATE;
			break;
		case TKFX_STATE_MODE_UPDATE:
			// Check supercap voltage.
			tkfx_ctx.mode = (tkfx_ctx.vcap_mv < ((tkfx_ctx.config) -> vcap_min_mv)) ? TKFX_MODE_LOW_POWER : TKFX_MODE_ACTIVE;
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_ACCELERO;
			break;
#ifdef SSM
		case TKFX_STATE_ACCELERO:
			IWDG_reload();
			// Configure accelerometer according to mode.
			if ((tkfx_ctx.mode == TKFX_MODE_ACTIVE) && ((tkfx_ctx.status.accelerometer_status == 0) || (tkfx_ctx.flags.por != 0))) {
				// Active mode.
				I2C1_power_on();
				MMA8653FC_write_config(&(mma8653_active_config[0]), MMA8653FC_ACTIVE_CONFIG_LENGTH);
				I2C1_power_off();
				// Update status.
				tkfx_ctx.status.accelerometer_status = 1;
			}
			if ((tkfx_ctx.mode == TKFX_MODE_LOW_POWER) && ((tkfx_ctx.status.accelerometer_status != 0) || (tkfx_ctx.flags.por != 0))) {
				// Sleep mode.
				I2C1_power_on();
				MMA8653FC_write_config(&(mma8653_sleep_config[0]), MMA8653FC_SLEEP_CONFIG_LENGTH);
				I2C1_power_off();
				// Update status.
				tkfx_ctx.status.accelerometer_status = 0;
			}
			// Compute next state.
			tkfx_ctx.state = (tkfx_ctx.flags.por != 0) ? TKFX_STATE_OFF : TKFX_STATE_MONITORING;
			break;
#endif
		case TKFX_STATE_MONITORING:
			IWDG_reload();
			// Build Sigfox frame.
			tkfx_ctx.sigfox_monitoring_data.tamb_degrees = tkfx_ctx.tamb_degrees;
			tkfx_ctx.sigfox_monitoring_data.tmcu_degrees = tkfx_ctx.tmcu_degrees;
			tkfx_ctx.sigfox_monitoring_data.vsrc_mv = tkfx_ctx.vsrc_mv;
			tkfx_ctx.sigfox_monitoring_data.vcap_mv = tkfx_ctx.vcap_mv;
			tkfx_ctx.sigfox_monitoring_data.vmcu_mv = tkfx_ctx.vmcu_mv;
			tkfx_ctx.sigfox_monitoring_data.status = tkfx_ctx.status.all;
			// Send uplink monitoring frame.
			sigfox_api_status = SIGFOX_API_open(&tkfx_ctx.sigfox_rc);
			if (sigfox_api_status == SFX_ERR_NONE) {
				sigfox_api_status = SIGFOX_API_send_frame(tkfx_ctx.sigfox_monitoring_data.frame, TKFX_SIGFOX_MONITORING_DATA_LENGTH, tkfx_ctx.sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Compute next state.
			tkfx_ctx.state = TKFX_GEOLOC_CHECK;
			break;
		case TKFX_GEOLOC_CHECK:
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
			tkfx_ctx.flags.geoloc_timeout = (neom8n_status == NEOM8N_SUCCESS) ? 0 : 1;
			// Build Sigfox frame.
			if (tkfx_ctx.flags.geoloc_timeout == 0) {
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
			}
			else {
				tkfx_ctx.sigfox_geoloc_data.frame[0] = tkfx_ctx.geoloc_fix_duration_seconds;
			}
			// Send uplink geolocation frame.
			sigfox_api_status = SIGFOX_API_open(&tkfx_ctx.sigfox_rc);
			if (sigfox_api_status == SFX_ERR_NONE) {
				sigfox_api_status = SIGFOX_API_send_frame(tkfx_ctx.sigfox_geoloc_data.frame, (tkfx_ctx.flags.geoloc_timeout ? TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH : TKFX_SIGFOX_GEOLOC_DATA_LENGTH), tkfx_ctx.sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Reset geoloc variables.
			tkfx_ctx.flags.geoloc_timeout = 0;
			tkfx_ctx.geoloc_fix_duration_seconds = 0;
			tkfx_ctx.geoloc_timer_seconds = 0,
			// Compute next state.
			tkfx_ctx.state = TKFX_STATE_OFF;
			break;
		case TKFX_STATE_OFF:
			IWDG_reload();
			// Clear POR flag.
			tkfx_ctx.flags.por = 0;
			// Reset algorithm variables.
			tkfx_ctx.start_detection_irq_count = 0;
			tkfx_ctx.stop_detection_timer_seconds = 0;
			tkfx_ctx.keep_alive_timer_seconds = 0;
			// Clear EXTI flags.
			EXTI_clear_all_flags();
			// Enable RTC interrupt.
			RTC_clear_wakeup_timer_flag();
			RTC_start_wakeup_timer(RTC_WAKEUP_PERIOD_SECONDS);
#ifdef SSM
			// Enable accelerometer interrupt if required.
			if (tkfx_ctx.status.accelerometer_status != 0) {
				// Enable interrupt.
				MMA8653FC_clear_motion_interrupt_flag();
				NVIC_enable_interrupt(NVIC_IT_EXTI_0_1);
			}
#endif
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
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init board.
	TKFX_init_context();
	TKFX_init_hw();
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

