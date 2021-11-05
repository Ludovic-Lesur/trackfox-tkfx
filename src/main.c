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

#ifdef SSM
#define TKFX_STOP_CONDITION_THRESHOLD_SECONDS			300
#define TKFX_KEEP_ALIVE_PERIOD_SECONDS					3600
#endif
#ifdef PM
#define TKFX_GEOLOC_PERIOD_SECONDS						120
#endif
#define TKFX_GEOLOC_TIMEOUT_SECONDS						180
#define TKFX_GEOLOC_VCAP_MIN_MV							1500
#define TKFX_SIGFOX_GEOLOC_DATA_LENGTH_BYTES			11
#define TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH_BYTES	1
#define TKFX_SIGFOX_DOWNLINK_DATA_LENGTH_BYTES			8
#define TKFX_SIGFOX_MONITORING_DATA_LENGTH_BYTES		8

/*** MAIN structures ***/

#ifndef ATM
// State machine.
typedef enum {
	TKFX_STATE_POR,
	TKFX_STATE_INIT,
#ifdef SSM
	TKFX_STATE_ACCELERO,
#endif
	TKFX_STATE_OOB,
	TKFX_STATE_MEASURE,
	TKFX_STATE_MONITORING,
	TKFX_STATE_GEOLOC,
	TKFX_STATE_OFF,
	TKFX_STATE_SLEEP
} TKFX_State;

// Sigfox monitoring frame data format.
typedef union {
	unsigned char raw_frame[TKFX_SIGFOX_MONITORING_DATA_LENGTH_BYTES];
	struct {
		unsigned tamb_degrees : 8;
		unsigned tmcu_degrees : 8;
		unsigned vsrc_mv : 16;
		unsigned vcap_mv : 12;
		unsigned vmcu_mv : 12;
		unsigned status_byte : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} TKFX_SigfoxMonitoringData;

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
} TKFX_SigfoxGeolocData;

// Status byte bit indexes.
typedef enum {
	TKFX_STATUS_BYTE_TRACKER_MODE0_BIT_IDX,
	TKFX_STATUS_BYTE_TRACKER_MODE1_BIT_IDX,
	TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX,
	TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX,
	TKFX_STATUS_BYTE_LSI_STATUS_BIT_IDX,
	TKFX_STATUS_BYTE_LSE_STATUS_BIT_IDX
} TKFX_StatusBitsIndex;

// Device context.
typedef struct {
	// State machine.
	TKFX_State tkfx_state;
	unsigned char tkfx_por_flag;
	unsigned int tkfx_lsi_frequency_hz;
#ifdef SSM
	unsigned int tkfx_stop_timer_seconds;
	unsigned int tkfx_keep_alive_timer_seconds;
#endif
#ifdef PM
	unsigned int tkfx_geoloc_timer_seconds;
#endif
	unsigned char tkfx_status_byte;
	// Monitoring data.
	unsigned char tkfx_tamb_degrees;
	unsigned char tkfx_tmcu_degrees;
	unsigned int tkfx_vsrc_mv;
	unsigned int tkfx_vcap_mv;
	unsigned int tkfx_vmcu_mv;
	// Geoloc.
	Position tkfx_geoloc_position;
	unsigned int tkfx_geoloc_fix_duration_seconds;
	unsigned int tkfx_geoloc_timeout;
	// Sigfox.
	TKFX_SigfoxMonitoringData tkfx_sfx_monitoring_data;
	TKFX_SigfoxGeolocData tkfx_sfx_geoloc_data;
	unsigned char tkfx_sfx_downlink_data[TKFX_SIGFOX_DOWNLINK_DATA_LENGTH_BYTES];
} TKFX_Context;
#endif

/*** MAIN global variables ***/

#ifndef ATM
static TKFX_Context tkfx_ctx;
#endif

/*** MAIN functions ***/

#ifndef ATM
/* MAIN FUNCTION FOR START/STOP MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init memory.
	NVIC_Init();
	// Init GPIOs.
	GPIO_Init();
	EXTI_Init();
	// Init clock and power modules.
	RCC_Init();
	PWR_Init();
	// Init context.
	tkfx_ctx.tkfx_por_flag = 1;
	tkfx_ctx.tkfx_lsi_frequency_hz = 0;
	tkfx_ctx.tkfx_state = TKFX_STATE_POR;
#ifdef SSM
	tkfx_ctx.tkfx_stop_timer_seconds = 0;
	tkfx_ctx.tkfx_keep_alive_timer_seconds = 0;
#endif
#ifdef PM
	tkfx_ctx.tkfx_geoloc_timer_seconds = 0;
#endif
	tkfx_ctx.tkfx_status_byte = 0; // Reset all flags and tracker mode='00'.
	// Local variables.
	unsigned char tkfx_use_lse = 0;
	unsigned char hse_success = 0;
	unsigned int sfx_error = 0;
	sfx_rc_t tkfx_sigfox_rc = (sfx_rc_t) RC1;
	unsigned int geoloc_fix_start_time_seconds = 0;
	NEOM8N_ReturnCode neom8n_return_code = NEOM8N_TIMEOUT;
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (tkfx_ctx.tkfx_state) {
		case TKFX_STATE_POR:
			// Init watchdog.
#ifndef DEBUG
			IWDG_Init();
#endif
			IWDG_Reload();
			// Reset RTC before starting oscillators.
			RTC_Reset();
			// Low speed oscillators.
			tkfx_ctx.tkfx_status_byte |= (RCC_EnableLsi() << TKFX_STATUS_BYTE_LSI_STATUS_BIT_IDX);
			tkfx_use_lse = RCC_EnableLse();
			// Switch to HSI clock.
			RCC_SwitchToHsi();
			// Get LSI effective frequency (must be called after HSI initialization and before RTC inititialization).
			RCC_GetLsiFrequency(&tkfx_ctx.tkfx_lsi_frequency_hz);
			IWDG_Reload();
			// Init RTC.
			RTC_Init(&tkfx_use_lse, tkfx_ctx.tkfx_lsi_frequency_hz);
			tkfx_ctx.tkfx_status_byte |= (tkfx_use_lse << TKFX_STATUS_BYTE_LSE_STATUS_BIT_IDX);
			// Compute next state.
			tkfx_ctx.tkfx_state = TKFX_STATE_INIT;
			break;
		case TKFX_STATE_INIT:
			IWDG_Reload();
			// Disable RTC interrupt.
			RTC_StopWakeUpTimer();
#ifdef SSM
			// Reset keep-alive timer.
			tkfx_ctx.tkfx_keep_alive_timer_seconds = 0;
			// Disable accelerometer interrupt.
			NVIC_DisableInterrupt(NVIC_IT_EXTI_0_1);
#endif
#ifdef PM
			// Reset geoloc timer.
			tkfx_ctx.tkfx_geoloc_timer_seconds = 0;
#endif
			// High speed oscillator.
			IWDG_Reload();
			RCC_SwitchToHsi();
			// Init delay timer.
			LPTIM1_Init(tkfx_ctx.tkfx_lsi_frequency_hz);
			// Unused communication interfaces.
			USART2_Init();
			// Init components.
			NEOM8N_Init();
			SHT3X_Init();
#ifdef SSM
			MMA8653FC_Init();
#endif
			// Compute next state.
			if (tkfx_ctx.tkfx_por_flag == 0) {
				tkfx_ctx.tkfx_state = TKFX_STATE_MEASURE;
			}
			else {
#ifdef SSM
				tkfx_ctx.tkfx_state = TKFX_STATE_ACCELERO;
#else
				tkfx_ctx.tkfx_state = TKFX_STATE_OOB;
#endif
			}
			break;
#ifdef SSM
		case TKFX_STATE_ACCELERO:
			IWDG_Reload();
			// Configure accelerometer once for motion detection.
			I2C1_Init();
			I2C1_PowerOn();
			MMA8653FC_WriteConfig(&(mma8653_tkfx_config[0]), MMA8653FC_TKFX_CONFIG_SIZE);
			I2C1_PowerOff();
			I2C1_Disable();
			// Compute next state.
			tkfx_ctx.tkfx_state = TKFX_STATE_OOB;
			break;
#endif
		case TKFX_STATE_OOB:
			IWDG_Reload();
			// Send OOB frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
			}
			SIGFOX_API_close();
			// Compute next state.
#ifdef SSM
			tkfx_ctx.tkfx_state = TKFX_STATE_OFF;
#else
			tkfx_ctx.tkfx_state = TKFX_STATE_GEOLOC;
#endif
			break;
		case TKFX_STATE_MEASURE:
			IWDG_Reload();
			// Get temperature from SHT30.
			I2C1_Init();
			I2C1_PowerOn();
			SHT3X_PerformMeasurements();
			I2C1_PowerOff();
			I2C1_Disable();
			SHT3X_GetTemperatureComp1(&tkfx_ctx.tkfx_tamb_degrees);
			// Get voltages measurements.
			ADC1_Init();
			ADC1_PowerOn();
			ADC1_PerformMeasurements();
			ADC1_PowerOff();
			ADC1_Disable();
			ADC1_GetData(ADC_DATA_IDX_VSRC_MV, &tkfx_ctx.tkfx_vsrc_mv);
			ADC1_GetData(ADC_DATA_IDX_VCAP_MV, &tkfx_ctx.tkfx_vcap_mv);
			ADC1_GetData(ADC_DATA_IDX_VMCU_MV, &tkfx_ctx.tkfx_vmcu_mv);
			ADC1_GetTmcuComp1(&tkfx_ctx.tkfx_tmcu_degrees);
			// Compute next state.
			tkfx_ctx.tkfx_state = TKFX_STATE_MONITORING;
			break;
		case TKFX_STATE_MONITORING:
			IWDG_Reload();
			// Build Sigfox frame.
			tkfx_ctx.tkfx_sfx_monitoring_data.field.tamb_degrees = tkfx_ctx.tkfx_tamb_degrees;
			tkfx_ctx.tkfx_sfx_monitoring_data.field.tmcu_degrees = tkfx_ctx.tkfx_tmcu_degrees;
			tkfx_ctx.tkfx_sfx_monitoring_data.field.vsrc_mv = tkfx_ctx.tkfx_vsrc_mv;
			tkfx_ctx.tkfx_sfx_monitoring_data.field.vcap_mv = tkfx_ctx.tkfx_vcap_mv;
			tkfx_ctx.tkfx_sfx_monitoring_data.field.vmcu_mv = tkfx_ctx.tkfx_vmcu_mv;
			tkfx_ctx.tkfx_sfx_monitoring_data.field.status_byte = tkfx_ctx.tkfx_status_byte;
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(tkfx_ctx.tkfx_sfx_monitoring_data.raw_frame, TKFX_SIGFOX_MONITORING_DATA_LENGTH_BYTES, tkfx_ctx.tkfx_sfx_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Compute next state.
#ifdef SSM
			if (((tkfx_ctx.tkfx_status_byte & (0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX)) == 0) && ((tkfx_ctx.tkfx_status_byte & (0b1 << TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX)) != 0)) {
				// Stop condition.
				tkfx_ctx.tkfx_state = TKFX_STATE_GEOLOC;
			}
			else {
				// Moving condition.
				tkfx_ctx.tkfx_state = TKFX_STATE_OFF;
			}
#else
			tkfx_ctx.tkfx_state = TKFX_STATE_GEOLOC;
#endif
			break;
		case TKFX_STATE_GEOLOC:
			IWDG_Reload();
			// Check supercap voltage.
			if (tkfx_ctx.tkfx_vcap_mv < TKFX_GEOLOC_VCAP_MIN_MV) {
				// Do not perform GPS fix.
				tkfx_ctx.tkfx_geoloc_fix_duration_seconds = 0;
				tkfx_ctx.tkfx_geoloc_timeout = 1;
			}
			else {
				// Get position from GPS.
				LPUART1_Init(tkfx_use_lse);
				LPUART1_PowerOn();
				neom8n_return_code = NEOM8N_GetPosition(&tkfx_ctx.tkfx_geoloc_position, TKFX_GEOLOC_TIMEOUT_SECONDS, TKFX_GEOLOC_VCAP_MIN_MV, &tkfx_ctx.tkfx_geoloc_fix_duration_seconds);
				LPUART1_PowerOff();
				LPUART1_Disable();
				// Parse result.
				if (neom8n_return_code != NEOM8N_SUCCESS) {
					tkfx_ctx.tkfx_geoloc_timeout = 1;
				}
			}
			IWDG_Reload();
			// Build Sigfox frame.
			if (tkfx_ctx.tkfx_geoloc_timeout == 0) {
				tkfx_ctx.tkfx_sfx_geoloc_data.field.latitude_degrees = tkfx_ctx.tkfx_geoloc_position.lat_degrees;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.latitude_minutes = tkfx_ctx.tkfx_geoloc_position.lat_minutes;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.latitude_seconds = tkfx_ctx.tkfx_geoloc_position.lat_seconds;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.latitude_north_flag = tkfx_ctx.tkfx_geoloc_position.lat_north_flag;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.longitude_degrees = tkfx_ctx.tkfx_geoloc_position.long_degrees;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.longitude_minutes = tkfx_ctx.tkfx_geoloc_position.long_minutes;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.longitude_seconds = tkfx_ctx.tkfx_geoloc_position.long_seconds;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.longitude_east_flag = tkfx_ctx.tkfx_geoloc_position.long_east_flag;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.altitude_meters = tkfx_ctx.tkfx_geoloc_position.altitude;
				tkfx_ctx.tkfx_sfx_geoloc_data.field.gps_fix_duration_seconds = tkfx_ctx.tkfx_geoloc_fix_duration_seconds;
			}
			else {
				tkfx_ctx.tkfx_sfx_geoloc_data.raw_frame[0] = tkfx_ctx.tkfx_geoloc_fix_duration_seconds;
			}
			// Send uplink geolocation frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(tkfx_ctx.tkfx_sfx_geoloc_data.raw_frame, (tkfx_ctx.tkfx_geoloc_timeout ? TKFX_SIGFOX_GEOLOC_TIMEOUT_DATA_LENGTH_BYTES : TKFX_SIGFOX_GEOLOC_DATA_LENGTH_BYTES), tkfx_ctx.tkfx_sfx_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Reset geoloc variables.
			tkfx_ctx.tkfx_geoloc_timeout = 0;
			tkfx_ctx.tkfx_geoloc_fix_duration_seconds = 0;
			// Compute next state
			tkfx_ctx.tkfx_state = TKFX_STATE_OFF;
			break;
		case TKFX_STATE_OFF:
			IWDG_Reload();
			// Clear POR flag.
			tkfx_ctx.tkfx_por_flag = 0;
			// Turn peripherals off.
			LPTIM1_Disable();
			// Clear EXTI flags.
			EXTI_ClearAllFlags();
			RTC_ClearWakeUpTimerFlag();
#ifdef SSM
			// Enable accelerometer interrupt.
			MMA8653FC_ClearMotionInterruptFlag();
			NVIC_EnableInterrupt(NVIC_IT_EXTI_0_1);
#endif
			// Enable RTC interrupt.
			RTC_StartWakeUpTimer(RTC_WAKEUP_PERIOD_SECONDS);
			// Enter stop mode.
			tkfx_ctx.tkfx_state = TKFX_STATE_SLEEP;
			break;
		case TKFX_STATE_SLEEP:
			IWDG_Reload();
			// Enter sleep mode.
			PWR_EnterStopMode();
			// Check wake-up source.
			if (RTC_GetWakeUpTimerFlag() != 0) {
#ifdef SSM
				// Increment timers.
				tkfx_ctx.tkfx_keep_alive_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
				tkfx_ctx.tkfx_stop_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
				// Check keep-alive period.
				if (tkfx_ctx.tkfx_keep_alive_timer_seconds >= TKFX_KEEP_ALIVE_PERIOD_SECONDS) {
#endif
#ifdef PM
				// Increment timer.
				tkfx_ctx.tkfx_geoloc_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
				// Check geolocation period.
				if (tkfx_ctx.tkfx_geoloc_timer_seconds >= TKFX_GEOLOC_PERIOD_SECONDS) {
#endif
					// Reset alarm flag.
					tkfx_ctx.tkfx_status_byte &= ~(0b1 << TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX);
					// Turn tracker on to send keep-alive.
					tkfx_ctx.tkfx_state = TKFX_STATE_INIT;
				}
				// Clear RTC flags.
				RTC_ClearWakeUpTimerFlag();
			}
#ifdef SSM
			if (MMA8653FC_GetMotionInterruptFlag() != 0) {
				// Reset stop timer.
				tkfx_ctx.tkfx_stop_timer_seconds = 0;
				// Wake-up from accelerometer interrupt.
				if ((tkfx_ctx.tkfx_status_byte & (0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX)) == 0) {
					// Start condition detected.
					tkfx_ctx.tkfx_status_byte |= (0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX);
					tkfx_ctx.tkfx_status_byte |= (0b1 << TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX);
					// Turn tracker on to send start alarm.
					tkfx_ctx.tkfx_state = TKFX_STATE_INIT;
				}
				MMA8653FC_ClearMotionInterruptFlag();
			}
			else {
				// No movement detected.
				if ((tkfx_ctx.tkfx_stop_timer_seconds >= TKFX_STOP_CONDITION_THRESHOLD_SECONDS) && ((tkfx_ctx.tkfx_status_byte & (0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX)) != 0)) {
					// Stop condition detected.
					tkfx_ctx.tkfx_status_byte &= ~(0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX);
					tkfx_ctx.tkfx_status_byte |= (0b1 << TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX);
					// Turn tracker on to send stop alarm.
					tkfx_ctx.tkfx_state = TKFX_STATE_INIT;
				}
			}
#endif
			break;
		default:
			// Unknown state.
			tkfx_ctx.tkfx_state = TKFX_STATE_OFF;
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
	RCC_EnableLsi();
#ifndef DEBUG
	IWDG_Init();
#endif
	// Init memory.
	NVIC_Init();
	// Init GPIOs.
	GPIO_Init();
	EXTI_Init();
	// Reset RTC before starting oscillators.
	RTC_Reset();
	// Init clocks.
	RCC_Init();
	unsigned char tkfx_use_lse = RCC_EnableLse();
	RCC_SwitchToHsi();
	// Get LSI effective frequency (must be called after HSI initialization and before RTC inititialization).
	unsigned int tkfx_lsi_frequency_hz = 0;
	RCC_GetLsiFrequency(&tkfx_lsi_frequency_hz);
	IWDG_Reload();
	// Init RTC and timers.
	RTC_Init(&tkfx_use_lse, tkfx_lsi_frequency_hz);
	LPTIM1_Init(tkfx_lsi_frequency_hz);
	// Init peripherals.
	ADC1_Init();
	USART2_Init();
	LPUART1_Init(tkfx_use_lse);
	// Components.
	NEOM8N_Init();
	SHT3X_Init();
	MMA8653FC_Init();
	// Configure accelerometer.
	I2C1_Init();
	I2C1_PowerOn();
	MMA8653FC_WriteConfig(&(mma8653_tkfx_config[0]), MMA8653FC_TKFX_CONFIG_SIZE);
	I2C1_PowerOff();
	I2C1_Disable();
	// Applicative layers.
	AT_Init();
	// Main loop.
	while (1) {
		AT_Task();
		IWDG_Reload();
	}
	return 0;
}
#endif

