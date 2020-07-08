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
#include "tim.h"
#include "usart.h"
// Components.
#include "mma8653fc.h"
#include "neom8n.h"
#include "s2lp.h"
#include "sht3x.h"
#include "sigfox_types.h"
// Applicative.
#include "at.h"
#include "dlk.h"
#include "geoloc.h"
#include "mode.h"
#include "monitoring.h"
#include "sigfox_api.h"

/*** MAIN macros ***/

#ifdef SSM
#define TKFX_STOP_CONDITION_THRESHOLD_SECONDS	300
#define TKFX_KEEP_ALIVE_PERIOD_SECONDS			86400
#define TKFX_GEOLOC_TIMEOUT_SECONDS				180
#endif

/*** MAIN structures ***/

#ifdef SSM
// State machine.
typedef enum {
	TKFX_STATE_POR,
	TKFX_STATE_INIT,
	TKFX_STATE_ACCELERO,
	TKFX_STATE_OOB,
	TKFX_STATE_MEASURE,
	TKFX_STATE_MONITORING,
	TKFX_STATE_GEOLOC,
	TKFX_STATE_OFF,
	TKFX_STATE_SLEEP
} TKFX_State;
#endif

#ifndef ATM
// Status byte bit indexes.
typedef enum {
	TKFX_STATUS_BYTE_TRACKER_MODE0_BIT_IDX,
	TKFX_STATUS_BYTE_TRACKER_MODE1_BIT_IDX,
	TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX,
	TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX,
	TKFX_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX,
	TKFX_STATUS_BYTE_LSI_STATUS_BIT_IDX,
	TKFX_STATUS_BYTE_LSE_STATUS_BIT_IDX,
	TKFX_STATUS_BYTE_DAILY_DOWNLINK_BIT_IDX
} TKFX_StatusBitsIndex;
#endif

#ifndef ATM
// Device context.
typedef struct {
	// State machine.
	TKFX_State tkfx_state;
	unsigned char tkfx_por_flag;
	unsigned int tkfx_stop_timer_seconds;
	unsigned int tkfx_keep_alive_timer_seconds;
	unsigned char tkfx_status_byte;
	// Monitoring.
	MONITORING_Data tkfx_monitoring_data;
	// Geoloc.
	Position tkfx_geoloc_position;
	unsigned int tkfx_geoloc_fix_duration_seconds;
	unsigned char tkfx_geoloc_timeout;
	// Sigfox.
	unsigned char tkfx_sfx_uplink_data[SFX_UPLINK_DATA_MAX_SIZE_BYTES];
	unsigned char tkfx_sfx_downlink_data[SFX_DOWNLINK_DATA_SIZE_BYTES];
} TKFX_Context;
#endif

/*** MAIN global variables ***/

#ifndef ATM
static TKFX_Context tkfx_ctx;
#endif

#ifdef SSM
/* MAIN FUNCTION FOR START/STOP MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init memory.
	NVIC_Init();
	FLASH_Init();
	NVM_Enable();
	// Init GPIOs.
	GPIO_Init();
	EXTI_Init();
	// Init clock module.
	RCC_Init();
	// Init context.
	tkfx_ctx.tkfx_por_flag = 1;
	tkfx_ctx.tkfx_state = TKFX_STATE_POR;
	tkfx_ctx.tkfx_stop_timer_seconds = 0;
	tkfx_ctx.tkfx_keep_alive_timer_seconds = 0;
	tkfx_ctx.tkfx_status_byte = 0; // Reset all flags and tracker mode='00'.
	// Local variables.
	unsigned char rtc_use_lse = 0;
	unsigned int sfx_error = 0;
	sfx_rc_t tkfx_sigfox_rc = (sfx_rc_t) RC1;
	unsigned int geoloc_fix_start_time_seconds = 0;
	NEOM8N_ReturnCode neom8n_return_code = NEOM8N_TIMEOUT;
	// Main loop.
	while (1) {
		// Perform state machine.
		switch (tkfx_ctx.tkfx_state) {
		case TKFX_STATE_POR:
			// LSI.
			tkfx_ctx.tkfx_status_byte &= ~(0b1 << TKFX_STATUS_BYTE_LSI_STATUS_BIT_IDX);
			tkfx_ctx.tkfx_status_byte |= (RCC_EnableLsi() << TKFX_STATUS_BYTE_LSI_STATUS_BIT_IDX);
			// Init watchdog.
			IWDG_Init();
			// Reset RTC.
			RTC_Reset();
			// LSE.
			tkfx_ctx.tkfx_status_byte &= ~(0b1 << TKFX_STATUS_BYTE_LSE_STATUS_BIT_IDX);
			tkfx_ctx.tkfx_status_byte |= (RCC_EnableLse() << TKFX_STATUS_BYTE_LSE_STATUS_BIT_IDX);
			// RTC.
			IWDG_Reload();
			if (tkfx_ctx.tkfx_por_flag != 0) {
				rtc_use_lse = tkfx_ctx.tkfx_status_byte & (0b1 << TKFX_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				RTC_Init(&rtc_use_lse);
				// Update LSE status if RTC failed to start on it.
				if (rtc_use_lse == 0) {
					tkfx_ctx.tkfx_status_byte &= ~(0b1 << TKFX_STATUS_BYTE_LSE_STATUS_BIT_IDX);
				}
			}
			IWDG_Reload();
			// Compute next state.
			tkfx_ctx.tkfx_state = TKFX_STATE_INIT;
			break;
		case TKFX_STATE_INIT:
			// Reset timers.
			tkfx_ctx.tkfx_stop_timer_seconds = 0;
			tkfx_ctx.tkfx_keep_alive_timer_seconds = 0;
			// High speed oscillator.
			IWDG_Reload();
			RCC_EnableGpio();
			tkfx_ctx.tkfx_status_byte &= ~(0b1 << TKFX_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			tkfx_ctx.tkfx_status_byte |= (RCC_SwitchToHse() << TKFX_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX);
			if ((tkfx_ctx.tkfx_status_byte & (0b1 << TKFX_STATUS_BYTE_MCU_CLOCK_SOURCE_BIT_IDX)) == 0) {
				RCC_SwitchToHsi();
			}
			// Init timers.
			TIM21_Init();
			TIM22_Init();
			TIM21_Start();
			TIM22_Start();
			LPTIM1_Init(0);
			// DMA.
			DMA1_InitChannel3();
			DMA1_InitChannel6();
			// Analog.
			ADC1_Init();
			// Communication interfaces.
			LPUART1_Init();
			USART2_Init();
			I2C1_Init();
			SPI1_Init();
			// Hardware AES.
			AES_Init();
			// Init components.
			NEOM8N_Init();
			SHT3X_Init();
			S2LP_Init();
			MMA8653FC_Init();
			// Compute next state.
			if (tkfx_ctx.tkfx_por_flag == 0) {
				tkfx_ctx.tkfx_state = TKFX_STATE_MEASURE;
			}
			else {
				tkfx_ctx.tkfx_state = TKFX_STATE_ACCELERO;
			}
			break;
		case TKFX_STATE_ACCELERO:
			// Configure accelerometer once for motion detection.
			I2C1_PowerOn();
			MMA8653FC_WriteConfig(&(mma8653_tkfx_config[0]), MMA8653FC_TKFX_CONFIG_SIZE);
			I2C1_PowerOff();
			// Compute next state.
			tkfx_ctx.tkfx_state = TKFX_STATE_OOB;
			break;
		case TKFX_STATE_OOB:
			IWDG_Reload();
			// Send OOB frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
			}
			SIGFOX_API_close();
			// Compute next state.
			tkfx_ctx.tkfx_state = TKFX_STATE_OFF;
			break;
		case TKFX_STATE_MEASURE:
			IWDG_Reload();
			// Get temperature from SHT30.
			I2C1_PowerOn();
			SHT3X_PerformMeasurements();
			I2C1_PowerOff();
			SHT3X_GetTemperature(&(tkfx_ctx.tkfx_monitoring_data.monitoring_data_temperature_degrees));
			// Get voltages measurements.
			ADC1_PowerOn();
			ADC1_PerformMeasurements();
			ADC1_PowerOff();
			ADC1_GetSupercapVoltage(&(tkfx_ctx.tkfx_monitoring_data.monitoring_data_supercap_voltage_mv));
			ADC1_GetSourceVoltage(&(tkfx_ctx.tkfx_monitoring_data.monitoring_data_source_voltage_mv));
			ADC1_GetMcuVoltage(&(tkfx_ctx.tkfx_monitoring_data.monitoring_data_mcu_voltage_mv));
			// Status byte.
			tkfx_ctx.tkfx_monitoring_data.monitoring_data_status_byte = tkfx_ctx.tkfx_status_byte;
			// Compute next state.
			tkfx_ctx.tkfx_state = TKFX_STATE_MONITORING;
			break;
		case TKFX_STATE_MONITORING:
			IWDG_Reload();
			// Build Sigfox frame.
			MONITORING_BuildSigfoxData(&tkfx_ctx.tkfx_monitoring_data, tkfx_ctx.tkfx_sfx_uplink_data);
			// Send uplink monitoring frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(tkfx_ctx.tkfx_sfx_uplink_data, MONITORING_SIGFOX_DATA_LENGTH, tkfx_ctx.tkfx_sfx_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Compute next state.
			if ((tkfx_ctx.tkfx_status_byte & (0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX)) == 0) {
				// Stop condition.
				tkfx_ctx.tkfx_state = TKFX_STATE_GEOLOC;
			}
			else {
				// Moving condition.
				tkfx_ctx.tkfx_state = TKFX_STATE_OFF;
			}
			break;
		case TKFX_STATE_GEOLOC:
			IWDG_Reload();
			// Get position from GPS.
			LPUART1_PowerOn();
			geoloc_fix_start_time_seconds = TIM22_GetSeconds();
			neom8n_return_code = NEOM8N_GetPosition(&tkfx_ctx.tkfx_geoloc_position, TKFX_GEOLOC_TIMEOUT_SECONDS);
			LPUART1_PowerOff();
			// Parse result.
			if (neom8n_return_code == NEOM8N_SUCCESS) {
				// Get fix duration and update flag.
				tkfx_ctx.tkfx_geoloc_fix_duration_seconds = TIM22_GetSeconds() - geoloc_fix_start_time_seconds;
				if (tkfx_ctx.tkfx_geoloc_fix_duration_seconds > TKFX_GEOLOC_TIMEOUT_SECONDS) {
					tkfx_ctx.tkfx_geoloc_fix_duration_seconds = TKFX_GEOLOC_TIMEOUT_SECONDS;
				}
			}
			else {
				// Set fix duration to timeout.
				tkfx_ctx.tkfx_geoloc_fix_duration_seconds = TKFX_GEOLOC_TIMEOUT_SECONDS;
				tkfx_ctx.tkfx_geoloc_timeout = 1;
			}
			IWDG_Reload();
			// Build Sigfox frame.
			GEOLOC_BuildSigfoxData(&tkfx_ctx.tkfx_geoloc_position, tkfx_ctx.tkfx_geoloc_fix_duration_seconds, tkfx_ctx.tkfx_geoloc_timeout, tkfx_ctx.tkfx_sfx_uplink_data);
			// Send uplink geolocation frame.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(tkfx_ctx.tkfx_sfx_uplink_data, (tkfx_ctx.tkfx_geoloc_timeout ? GEOLOC_TIMEOUT_SIGFOX_DATA_LENGTH : GEOLOC_SIGFOX_DATA_LENGTH), tkfx_ctx.tkfx_sfx_downlink_data, 2, 0);
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
			S2LP_DisableGpio();
			ADC1_Disable();
			TIM21_Disable();
			TIM22_Disable();
			LPTIM1_Disable();
			SPI1_Disable();
			DMA1_Disable();
			LPUART1_Disable();
			I2C1_Disable();
			AES_Disable();
			NVM_Disable();
			// Switch to internal MSI 65kHz (must be called before WIND functions to init TIM2 with right clock frequency).
			RCC_SwitchToMsi();
			RCC_DisableGpio();
			// Clear EXTI flags.
			EXTI_ClearAllFlags();
			RTC_ClearAlarmAFlag();
			MMA8653FC_ClearMotionInterruptFlag();
			// Enable RTC and accelerometer interrupts.
			NVIC_EnableInterrupt(IT_RTC);
			NVIC_EnableInterrupt(IT_EXTI_0_1);
			// Enter stop mode.
			tkfx_ctx.tkfx_state = TKFX_STATE_SLEEP;
			break;
		case TKFX_STATE_SLEEP:
			IWDG_Reload();
			// Enter sleep mode.
			PWR_EnterStopMode();
			// Wake-up. Reload watchdog.
			IWDG_Reload();
			// Increment timers.
			tkfx_ctx.tkfx_keep_alive_timer_seconds++;
			tkfx_ctx.tkfx_stop_timer_seconds++;
			// Check wake-up source.
			if (RTC_GetAlarmAFlag() != 0) {
				// Keep alive period reached.
				tkfx_ctx.tkfx_status_byte &= ~(0b1 << TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX);
				// Turn tracker on to send keep-alive.
				tkfx_ctx.tkfx_state = TKFX_STATE_INIT;
			}
			if (MMA8653FC_GetMotionInterruptFlag() != 0) {
				// Wake-up from accelerometer interrupt.
				if ((tkfx_ctx.tkfx_status_byte & (0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX)) == 0) {
					// Start condition detected.
					tkfx_ctx.tkfx_status_byte |= (0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX);
					tkfx_ctx.tkfx_status_byte |= (0b1 << TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX);
					// Turn tracker on to send start alarm.
					tkfx_ctx.tkfx_state = TKFX_STATE_INIT;
				}
				else {
					// Reset stop timer.
					tkfx_ctx.tkfx_stop_timer_seconds = 0;
				}
			}
			else {
				// No movement detected.
				if (tkfx_ctx.tkfx_stop_timer_seconds > TKFX_STOP_CONDITION_THRESHOLD_SECONDS) {
					// Stop condition detected.
					tkfx_ctx.tkfx_status_byte &= ~(0b1 << TKFX_STATUS_BYTE_MOVING_FLAG_BIT_IDX);
					tkfx_ctx.tkfx_status_byte |= (0b1 << TKFX_STATUS_BYTE_ALARM_FLAG_BIT_IDX);
					// Turn tracker on to send stop alarm.
					tkfx_ctx.tkfx_state = TKFX_STATE_INIT;
				}
			}
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
	// Init memory.
	NVIC_Init();
	FLASH_Init();
	NVM_Enable();
	// Init GPIOs.
	GPIO_Init();
	// Init clocks.
	RCC_Init();
	RCC_SwitchToHsi();
	// Timers.
	TIM21_Init();
	TIM22_Init();
	TIM21_Start();
	TIM22_Start();
	LPTIM1_Init(0);
	// DMA.
	DMA1_InitChannel3();
	DMA1_InitChannel6();
	// Analog.
	ADC1_Init();
	// External interrupts.
	EXTI_Init();
	// Communication interfaces.
	LPUART1_Init();
	USART2_Init();
	I2C1_Init();
	SPI1_Init();
	// Hardware AES.
	AES_Init();
	// Components.
	NEOM8N_Init();
	SHT3X_Init();
	S2LP_Init();
	MMA8653FC_Init();
	// Configure accelerometer.
	I2C1_PowerOn();
	MMA8653FC_WriteConfig(&(mma8653_tkfx_config[0]), MMA8653FC_TKFX_CONFIG_SIZE);
	I2C1_PowerOff();
	// Applicative layers.
	AT_Init();
	// Main loop.
	while (1) {
		AT_Task();
	}
	return 0;
}
#endif
