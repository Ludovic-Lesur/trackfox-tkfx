/*
 * tim.c
 *
 *  Created on: 03 jul. 2021
 *      Author: Ludo
 */

#include "tim.h"

#include "iwdg.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"
#include "types.h"

/*** TIM local macros ***/

#define TIM_TIMEOUT_COUNT				1000000

#define TIM2_CNT_VALUE_MAX				0xFFFF

#define TIM2_TARGET_TRIGGER_CLOCK_HZ	2048

#define TIM2_PRESCALER_ETRF_LSE			1
#define TIM2_PRESCALER_PSC_LSE			((RCC_LSE_FREQUENCY_HZ) / (TIM2_TARGET_TRIGGER_CLOCK_HZ * TIM2_PRESCALER_ETRF_LSE))

#define TIM2_PRESCALER_ETRF_HSI			8
#define TIM2_PRESCAKER_PSC_HSI			((RCC_HSI_FREQUENCY_KHZ * 1000) / (TIM2_TARGET_TRIGGER_CLOCK_HZ * TIM2_PRESCALER_ETRF_HSI))

#define TIM2_CLOCK_SWITCH_LATENCY_MS	2

#define TIM2_TIMER_DURATION_MS_MIN		1
#define TIM2_TIMER_DURATION_MS_MAX		((TIM2_CNT_VALUE_MAX * 1000) / (tim2_ctx.etrf_clock_hz))

/*** TIM local structures ***/

/*******************************************************************/
typedef enum {
	TIM2_TRIGGER_SOURCE_LSE = 0,
	TIM2_TRIGGER_SOURCE_HSI,
	TIM2_TRIGGER_SOURCE_LAST
} TIM2_trigger_source_t;

/*******************************************************************/
typedef struct {
	TIM2_trigger_source_t trigger_source;
	uint32_t etrf_clock_hz;
	volatile uint8_t channel_running[TIM2_CHANNEL_LAST];
} TIM2_context_t;

/*** TIM local global variables ***/

static TIM2_context_t tim2_ctx;
static volatile uint8_t tim21_flag = 0;

/*** TIM local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM2_IRQHandler(void) {
	// Local variables.
	uint8_t channel_idx = 0;
	// Channels loop.
	for (channel_idx=0 ; channel_idx<TIM2_CHANNEL_LAST ; channel_idx++) {
		// Check flag.
		if (((TIM2 -> SR) & (0b1 << (channel_idx + 1))) != 0) {
			// Reset flag.
			tim2_ctx.channel_running[channel_idx] = 0;
			// Clear flag.
			TIM2 -> SR &= ~(0b1 << (channel_idx + 1));
		}
	}
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM21_IRQHandler(void) {
	// TI1 interrupt.
	if (((TIM21 -> SR) & (0b1 << 1)) != 0) {
		// Update flags.
		if (((TIM21 -> DIER) & (0b1 << 1)) != 0) {
			tim21_flag = 1;
		}
		TIM21 -> SR &= ~(0b1 << 1);
	}
}

/*** TIM functions ***/

/*******************************************************************/
void TIM2_init(void) {
	// Local variables.
	uint8_t channel_idx = 0;
	// Init context.
	for (channel_idx=0 ; channel_idx<TIM2_CHANNEL_LAST ; channel_idx++) {
		tim2_ctx.channel_running[channel_idx] = 0;
	}
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 0); // TIM2EN='1'.
	RCC -> APB1SMENR |= (0b1 << 0); // TIM2SMEN='1'.
	// Select trigger.
	if (RCC_get_lse_status() != 0) {
		// Use LSE as trigger.
		RCC -> CR &= ~(0b1 << 5); // HSI16OUTEN='0'.
		TIM2 -> SMCR |= (0b00 << 12); // No prescaler on ETRF.
		TIM2 -> PSC = (TIM2_PRESCALER_PSC_LSE - 1);
		TIM2 -> OR |= (0b101 << 0);
		// Update context.
		tim2_ctx.trigger_source = TIM2_TRIGGER_SOURCE_LSE;
		tim2_ctx.etrf_clock_hz = ((RCC_LSE_FREQUENCY_HZ) / (TIM2_PRESCALER_ETRF_LSE * TIM2_PRESCALER_PSC_LSE));
	}
	else {
		// Use HSI as trigger.
		RCC -> CR |= (0b1 << 5); // HSI16OUTEN='1'.
		TIM2 -> SMCR |= (0b11 << 12); // ETRF prescaler = 8 (minimum 4 due to CK_INT clock ratio constraint).
		TIM2 -> PSC = (TIM2_PRESCAKER_PSC_HSI - 1);
		TIM2 -> OR |= (0b011 << 0);
		// Update context.
		tim2_ctx.trigger_source = TIM2_TRIGGER_SOURCE_HSI;
		tim2_ctx.etrf_clock_hz = ((RCC_HSI_FREQUENCY_KHZ * 1000) / (TIM2_PRESCALER_ETRF_HSI * TIM2_PRESCAKER_PSC_HSI));
	}
	// Use external clock mode 2.
	TIM2 -> SMCR |= (0b1 << 14) | (0b111 << 4);
	// Configure channels 1-4 in output compare mode.
	TIM2 -> CCMR1 &= 0xFFFF0000;
	TIM2 -> CCMR2 &= 0xFFFF0000;
	TIM2 -> CCER &= 0xFFFF0000;
	// Generate event to update registers.
	TIM2 -> EGR |= (0b1 << 0); // UG='1'.
	// Enable interrupt.
	NVIC_enable_interrupt(NVIC_INTERRUPT_TIM2, NVIC_PRIORITY_TIM2);
}

/*******************************************************************/
void TIM2_de_init(void) {
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_TIM2);
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 0); // TIM2EN='0'.
}

/*******************************************************************/
TIM_status_t TIM2_start(TIM2_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint32_t compare_value = 0;
	uint32_t local_duration_ms = duration_ms;
	uint32_t duration_min_ms = TIM2_TIMER_DURATION_MS_MIN;
	// Check parameters.
	if (waiting_mode >= TIM_WAITING_MODE_LAST) {
		status = TIM_ERROR_WAITING_MODE;
		goto errors;
	}
	// Check waiting mode.
	if (waiting_mode == TIM_WAITING_MODE_LOW_POWER_SLEEP) {
		// Compensate clock switch latency.
		duration_min_ms += TIM2_CLOCK_SWITCH_LATENCY_MS;
	}
	// Check parameters.
	if (channel >= TIM2_CHANNEL_LAST) {
		status = TIM_ERROR_CHANNEL;
		goto errors;
	}
	if (duration_ms < duration_min_ms) {
		status = TIM_ERROR_DURATION_UNDERFLOW;
		goto errors;
	}
	if (duration_ms > TIM2_TIMER_DURATION_MS_MAX) {
		status = TIM_ERROR_DURATION_OVERFLOW;
		goto errors;
	}
	// Compute compare value.
	if (waiting_mode == TIM_WAITING_MODE_LOW_POWER_SLEEP) {
		local_duration_ms -= TIM2_CLOCK_SWITCH_LATENCY_MS;
	}
	compare_value = ((TIM2 -> CNT) + ((local_duration_ms * tim2_ctx.etrf_clock_hz) / (1000))) % TIM2_CNT_VALUE_MAX;
	TIM2 -> CCRx[channel] = compare_value;
	// Update flag.
	tim2_ctx.channel_running[channel] = 1;
	// Clear flag.
	TIM2 -> SR &= ~(0b1 << (channel + 1));
	// Enable channel.
	TIM2 -> DIER |= (0b1 << (channel + 1));
	TIM2 -> CCER |= (0b1 << (4 * channel));
	// Enable counter.
	TIM2 -> CR1 |= (0b1 << 0);
errors:
	return status;
}

/*******************************************************************/
TIM_status_t TIM2_stop(TIM2_channel_t channel) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint8_t channel_idx = 0;
	uint8_t running_count = 0;
	// Check parameter.
	if (channel >= TIM2_CHANNEL_LAST) {
		status = TIM_ERROR_CHANNEL;
		goto errors;
	}
	// Disable channel.
	TIM2 -> CCER &= ~(0b1 << (4 * channel));
	TIM2 -> DIER &= ~(0b1 << (channel + 1));
	// Clear flag.
	TIM2 -> SR &= ~(0b1 << (channel + 1));
	// Update flag.
	tim2_ctx.channel_running[channel] = 0;
	// Disable counter if all channels are stopped.
	for (channel_idx=0 ; channel_idx<TIM2_CHANNEL_LAST ; channel_idx++) {
		running_count += tim2_ctx.channel_running[channel_idx];
	}
	if (running_count == 0) {
		TIM2 -> CR1 &= ~(0b1 << 0);
	}
errors:
	return status;
}

/*******************************************************************/
TIM_status_t TIM2_get_status(TIM2_channel_t channel, uint8_t* timer_has_elapsed) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	// Check parameters.
	if (channel >= TIM2_CHANNEL_LAST) {
		status = TIM_ERROR_CHANNEL;
		goto errors;
	}
	if (timer_has_elapsed == NULL) {
		status = TIM_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Update flag.
	(*timer_has_elapsed) = (tim2_ctx.channel_running[channel] == 0) ? 1 : 0;
errors:
	return status;
}

/*******************************************************************/
TIM_status_t TIM2_wait_completion(TIM2_channel_t channel, TIM_waiting_mode_t waiting_mode) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	// Check parameters.
	if (channel >= TIM2_CHANNEL_LAST) {
		status = TIM_ERROR_CHANNEL;
		goto errors;
	}
	// Sleep until channel is not running.
	switch (waiting_mode) {
	case TIM_WAITING_MODE_ACTIVE:
		// Active loop.
		while (tim2_ctx.channel_running[channel] != 0) {
			IWDG_reload();
		}
		break;
	case TIM_WAITING_MODE_SLEEP:
		// Enter sleep mode.
		while (tim2_ctx.channel_running[channel] != 0) {
			PWR_enter_sleep_mode();
			IWDG_reload();
		}
		break;
	case TIM_WAITING_MODE_LOW_POWER_SLEEP:
		// Check trigger source.
		if (tim2_ctx.trigger_source == TIM2_TRIGGER_SOURCE_LSE) {
			// Switch to MSI.
			rcc_status = RCC_switch_to_msi(RCC_MSI_RANGE_1_131KHZ);
			RCC_exit_error(TIM_ERROR_BASE_RCC);
			// Enter low power sleep mode.
			while (tim2_ctx.channel_running[channel] != 0) {
				PWR_enter_low_power_sleep_mode();
				IWDG_reload();
			}
			// Go back to HSI.
			rcc_status = RCC_switch_to_hsi();
			RCC_exit_error(TIM_ERROR_BASE_RCC);
		}
		else {
			// Enter sleep mode.
			while (tim2_ctx.channel_running[channel] != 0) {
				PWR_enter_sleep_mode();
				IWDG_reload();
			}
		}
		break;
	default:
		status = TIM_ERROR_WAITING_MODE;
		goto errors;
	}
errors:
	return status;
}

/*******************************************************************/
void TIM21_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.
	// Configure timer.
	// Channel input on TI1.
	// Capture done every 8 edges.
	// CH1 mapped on LSI.
	TIM21 -> CCMR1 |= (0b01 << 0) | (0b11 << 2);
	TIM21 -> OR |= (0b101 << 2);
	// Enable interrupt.
	TIM21 -> DIER |= (0b1 << 1); // CC1IE='1'.
	// Generate event to update registers.
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
}

/*******************************************************************/
void TIM21_de_init(void) {
	// Disable timer.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 2); // TIM21EN='0'.
}

/*******************************************************************/
TIM_status_t TIM21_measure_lsi_frequency(uint32_t* lsi_frequency_hz) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint8_t tim21_interrupt_count = 0;
	uint32_t tim21_ccr1_edge1 = 0;
	uint32_t tim21_ccr1_edge8 = 0;
	uint32_t loop_count = 0;
	// Check parameters.
	if (lsi_frequency_hz == NULL) {
		status = TIM_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset counter.
	TIM21 -> CNT = 0;
	TIM21 -> CCR1 = 0;
	// Enable interrupt.
	TIM21 -> SR &= 0xFFFFF9B8; // Clear all flags.
	NVIC_enable_interrupt(NVIC_INTERRUPT_TIM21, NVIC_PRIORITY_TIM21);
	// Enable TIM21 peripheral.
	TIM21 -> CR1 |= (0b1 << 0); // CEN='1'.
	TIM21 -> CCER |= (0b1 << 0); // CC1E='1'.
	// Wait for 2 captures.
	while (tim21_interrupt_count < 2) {
		// Wait for interrupt.
		tim21_flag = 0;
		loop_count = 0;
		while (tim21_flag == 0) {
			loop_count++;
			if (loop_count > TIM_TIMEOUT_COUNT) {
				status = TIM_ERROR_INTERRUPT_TIMEOUT;
				goto errors;
			}
		}
		tim21_interrupt_count++;
		if (tim21_interrupt_count == 1) {
			tim21_ccr1_edge1 = (TIM21 -> CCR1);
		}
		else {
			tim21_ccr1_edge8 = (TIM21 -> CCR1);
		}
	}
	// Compute LSI frequency.
	(*lsi_frequency_hz) = (8 * RCC_HSI_FREQUENCY_KHZ * 1000) / (tim21_ccr1_edge8 - tim21_ccr1_edge1);
errors:
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_TIM21);
	// Stop counter.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM21 -> CCER &= ~(0b1 << 0); // CC1E='0'.
	return status;
}
