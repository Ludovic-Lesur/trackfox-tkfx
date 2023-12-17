/*!*****************************************************************
 * \file    rf_api.c
 * \brief   Radio drivers.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "manuf/rf_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#include "error.h"
#include "iwdg.h"
#include "manuf/mcu_api.h"
#include "power.h"
#include "pwr.h"
#include "s2lp.h"

/*** RF API local macros ***/

#define RF_API_SYMBOL_PROFILE_SIZE_BYTES		40
#define RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES	(2 * RF_API_SYMBOL_PROFILE_SIZE_BYTES) // Size is twice to store PA and FDEV values.

#define RF_API_POLAR_DATARATE_MULTIPLIER		8

#define RF_API_FDEV_NEGATIVE					0x7F // FDEV * (+1)
#define RF_API_FDEV_POSITIVE					0x81 // FDEV * (-1)

#define RF_API_FIFO_BUFFER_FDEV_IDX				(RF_API_SYMBOL_PROFILE_SIZE_BYTES / 2) // Index where deviation is performed to invert phase.

#define RF_API_FIFO_TX_ALMOST_EMPTY_THRESHOLD	(RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES / 2)

#define RF_API_SMPS_FREQUENCY_HZ_TX				5500000
#ifdef BIDIRECTIONAL
#define RF_API_SMPS_FREQUENCY_HZ_RX				1500000
#endif

#ifdef BIDIRECTIONAL
#define RF_API_DL_PR_SIZE_BITS					32
#define RF_API_RX_BANDWIDTH_HZ					3000
#define RF_API_DOWNLINK_RSSI_THRESHOLD_DBM		-139
#endif

static const sfx_u8 RF_API_RAMP_AMPLITUDE_PROFILE[RF_API_SYMBOL_PROFILE_SIZE_BYTES] =
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 5, 7, 10, 14, 19, 25, 31, 39, 60, 220};
static const sfx_u8 RF_API_BIT0_AMPLITUDE_PROFILE[RF_API_SYMBOL_PROFILE_SIZE_BYTES] =
	{1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 5, 7, 10, 14, 19, 25, 31, 39, 60, 220, 220, 60, 39, 31, 25, 19, 14, 10, 7, 5, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1};
#ifdef BIDIRECTIONAL
static const sfx_u8 RF_API_DL_FT[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
#endif

/*** RF API local structures ***/

/*******************************************************************/
typedef enum {
	// Driver errors.
	RF_API_ERROR_NULL_PARAMETER = (RF_API_SUCCESS + 1),
	RF_API_ERROR_BUFFER_SIZE,
	RF_API_ERROR_STATE,
	RF_API_ERROR_MODULATION,
	RF_API_ERROR_MODE,
	RF_API_ERROR_LATENCY_TYPE,
	// Low level drivers errors.
	RF_API_ERROR_DRIVER_MCU_API,
	RF_API_ERROR_DRIVER_POWER,
	RF_API_ERROR_DRIVER_S2LP
} RF_API_custom_status_t;

/*******************************************************************/
typedef enum {
	RF_API_STATE_READY = 0,
	RF_API_STATE_TX_RAMP_UP,
	RF_API_STATE_TX_BITSTREAM,
	RF_API_STATE_TX_RAMP_DOWN,
	RF_API_STATE_TX_PADDING_BIT,
	RF_API_STATE_TX_END,
#ifdef BIDIRECTIONAL
	RF_API_STATE_RX_START,
	RF_API_STATE_RX,
#endif
	RF_API_STATE_LAST
} RF_API_state_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned gpio_irq_enable : 1;
		unsigned gpio_irq_flag : 1;
	} field;
	sfx_u8 all;
} RF_API_flags_t;

/*******************************************************************/
typedef struct {
	// Common.
	RF_API_state_t state;
	volatile RF_API_flags_t flags;
	// TX.
	sfx_u8 symbol_fifo_buffer[RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES];
	sfx_u8 ramp_fifo_buffer[RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES];
	sfx_u8 tx_bitstream[SIGFOX_UL_BITSTREAM_SIZE_BYTES];
	sfx_u8 tx_bitstream_size_bytes;
	sfx_u8 tx_byte_idx;
	sfx_u8 tx_bit_idx;
	sfx_u8 tx_fdev;
#ifdef BIDIRECTIONAL
	// RX.
	sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
	sfx_s16 dl_rssi_dbm;
#endif
} RF_API_context_t;

/*** RF API local global variables ***/

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
static sfx_u32 RF_API_LATENCY_MS[RF_API_LATENCY_LAST] = {
	0, // Wake-up.
	(POWER_ON_DELAY_MS_RADIO + S2LP_SHUTDOWN_DELAY_MS + 1), // TX init (power on delay + 1.75ms).
	0, // Send start (depends on bit rate and will be computed during init function).
	0, // Send stop (depends on bit rate and will be computed during init function).
	0, // TX de-init (70µs).
	0, // Sleep.
#ifdef BIDIRECTIONAL
	(POWER_ON_DELAY_MS_RADIO + S2LP_SHUTDOWN_DELAY_MS + 6), // RX init (power on delay + 5.97ms).
	0, // Receive start (300µs).
	7, // Receive stop (6.7ms).
	0, // RX de-init (70µs).
#endif
};
#endif
static RF_API_context_t rf_api_ctx;

/*** RF API local functions ***/

/*******************************************************************/
static void _RF_API_s2lp_gpio_irq_callback(void) {
	// Set flag if IRQ is enabled.
	rf_api_ctx.flags.field.gpio_irq_flag = rf_api_ctx.flags.field.gpio_irq_enable;
}

/*******************************************************************/
static RF_API_status_t _RF_API_internal_process(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	sfx_u8 s2lp_irq_flag = 0;
	sfx_u8 idx = 0;
	// Perform state machine.
	switch (rf_api_ctx.state) {
	case RF_API_STATE_READY:
		// Nothing to do.
		break;
	case RF_API_STATE_TX_RAMP_UP:
		// Fill ramp-up.
		for (idx=0 ; idx<RF_API_SYMBOL_PROFILE_SIZE_BYTES ; idx++) {
			rf_api_ctx.ramp_fifo_buffer[(2 * idx)] = 0; // Deviation.
			rf_api_ctx.ramp_fifo_buffer[(2 * idx) + 1] = RF_API_RAMP_AMPLITUDE_PROFILE[RF_API_SYMBOL_PROFILE_SIZE_BYTES - idx - 1]; // PA output power.
		}
		// Load ramp-up buffer into FIFO.
		s2lp_status = S2LP_send_command(S2LP_COMMAND_FLUSHTXFIFO);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_write_fifo((sfx_u8*) rf_api_ctx.ramp_fifo_buffer, RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Enable external GPIO interrupt.
		s2lp_status = S2LP_clear_all_irq();
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		rf_api_ctx.flags.field.gpio_irq_enable = 1;
		// Lock PLL.
		s2lp_status = S2LP_send_command(S2LP_COMMAND_LOCKTX);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_wait_for_state(S2LP_STATE_LOCK);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Start radio.
		s2lp_status = S2LP_send_command(S2LP_COMMAND_TX);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_wait_for_state(S2LP_STATE_TX);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Update state.
		rf_api_ctx.state = RF_API_STATE_TX_BITSTREAM;
		break;
	case RF_API_STATE_TX_BITSTREAM:
		// Read FIFO flag.
		s2lp_status = S2LP_get_irq_flag(S2LP_IRQ_INDEX_TX_FIFO_ALMOST_EMPTY, &s2lp_irq_flag);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Check flag.
		if (s2lp_irq_flag != 0) {
			// Check bit.
			if ((rf_api_ctx.tx_bitstream[rf_api_ctx.tx_byte_idx] & (1 << (7 - rf_api_ctx.tx_bit_idx))) == 0) {
				// Phase shift and amplitude shaping required.
				rf_api_ctx.tx_fdev = (rf_api_ctx.tx_fdev == RF_API_FDEV_NEGATIVE) ? RF_API_FDEV_POSITIVE : RF_API_FDEV_NEGATIVE; // Toggle deviation.
				for (idx=0 ; idx<RF_API_SYMBOL_PROFILE_SIZE_BYTES ; idx++) {
					rf_api_ctx.symbol_fifo_buffer[(2 * idx)] = (idx == RF_API_FIFO_BUFFER_FDEV_IDX) ? rf_api_ctx.tx_fdev : 0; // Deviation.
					rf_api_ctx.symbol_fifo_buffer[(2 * idx) + 1] = RF_API_BIT0_AMPLITUDE_PROFILE[idx]; // PA output power.
				}
			}
			else {
				// Constant CW.
				for (idx=0 ; idx<RF_API_SYMBOL_PROFILE_SIZE_BYTES ; idx++) {
					rf_api_ctx.symbol_fifo_buffer[(2 * idx)] = 0; // Deviation.
					rf_api_ctx.symbol_fifo_buffer[(2 * idx) + 1] = RF_API_BIT0_AMPLITUDE_PROFILE[0]; // PA output power.
				}
			}
			// Load bit into FIFO.
			s2lp_status = S2LP_write_fifo((sfx_u8*) rf_api_ctx.symbol_fifo_buffer, RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES);
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			// Increment bit index.
			rf_api_ctx.tx_bit_idx++;
			if (rf_api_ctx.tx_bit_idx >= 8) {
				// Reset bit index.
				rf_api_ctx.tx_bit_idx = 0;
				// Increment byte index.
				rf_api_ctx.tx_byte_idx++;
				// Check end of bitstream.
				if (rf_api_ctx.tx_byte_idx >= (rf_api_ctx.tx_bitstream_size_bytes)) {
					rf_api_ctx.tx_byte_idx = 0;
					// Update state.
					rf_api_ctx.state = RF_API_STATE_TX_RAMP_DOWN;
				}
			}
			// Clear flag.
			s2lp_status = S2LP_clear_all_irq();
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		}
		break;
	case RF_API_STATE_TX_RAMP_DOWN:
		// Read FIFO flag.
		s2lp_status = S2LP_get_irq_flag(S2LP_IRQ_INDEX_TX_FIFO_ALMOST_EMPTY, &s2lp_irq_flag);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Check flag.
		if (s2lp_irq_flag != 0) {
			// Fill ramp-down.
			for (idx=0 ; idx<RF_API_SYMBOL_PROFILE_SIZE_BYTES ; idx++) {
				rf_api_ctx.ramp_fifo_buffer[(2 * idx)] = 0; // FDEV.
				rf_api_ctx.ramp_fifo_buffer[(2 * idx) + 1] = RF_API_RAMP_AMPLITUDE_PROFILE[idx]; // PA output power for ramp-down.
			}
			// Load ramp-down buffer into FIFO.
			s2lp_status = S2LP_write_fifo((sfx_u8*) rf_api_ctx.ramp_fifo_buffer, RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES);
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			// Update state.
			rf_api_ctx.state = RF_API_STATE_TX_PADDING_BIT;
			// Clear flag.
			s2lp_status = S2LP_clear_all_irq();
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		}
		break;
	case RF_API_STATE_TX_PADDING_BIT:
		// Read FIFO flag.
		s2lp_status = S2LP_get_irq_flag(S2LP_IRQ_INDEX_TX_FIFO_ALMOST_EMPTY, &s2lp_irq_flag);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Check flag.
		if (s2lp_irq_flag != 0) {
			// Padding bit to ensure last ramp down is completely transmitted.
			for (idx=0 ; idx<RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES ; idx++) {
				rf_api_ctx.symbol_fifo_buffer[idx] = 0x00;
			}
			// Load padding buffer into FIFO.
			s2lp_status = S2LP_write_fifo((sfx_u8*) rf_api_ctx.symbol_fifo_buffer, RF_API_SYMBOL_FIFO_BUFFER_SIZE_BYTES);
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			// Update state.
			rf_api_ctx.state = RF_API_STATE_TX_END;
			// Clear flag.
			s2lp_status = S2LP_clear_all_irq();
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		}
		break;
	case RF_API_STATE_TX_END:
		// Read FIFO flag.
		s2lp_status = S2LP_get_irq_flag(S2LP_IRQ_INDEX_TX_FIFO_ALMOST_EMPTY, &s2lp_irq_flag);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Check flag.
		if (s2lp_irq_flag != 0) {
			// Stop radio.
			s2lp_status = S2LP_send_command(S2LP_COMMAND_SABORT);
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			// Disable interrupt.
			rf_api_ctx.flags.field.gpio_irq_enable = 0;
			s2lp_status = S2LP_clear_all_irq();
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			// Update state.
			rf_api_ctx.state = RF_API_STATE_READY;
		}
		break;
#ifdef BIDIRECTIONAL
	case RF_API_STATE_RX_START:
		// Flush FIFO.
		s2lp_status = S2LP_send_command(S2LP_COMMAND_FLUSHRXFIFO);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Enable external GPIO interrupt.
		s2lp_status = S2LP_clear_all_irq();
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		rf_api_ctx.flags.field.gpio_irq_enable = 1;
		// Lock PLL.
		s2lp_status = S2LP_send_command(S2LP_COMMAND_LOCKRX);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_wait_for_state(S2LP_STATE_LOCK);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Start radio.
		s2lp_status = S2LP_send_command(S2LP_COMMAND_RX);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_wait_for_state(S2LP_STATE_RX);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Update state.
		rf_api_ctx.state = RF_API_STATE_RX;
		break;
	case RF_API_STATE_RX:
		// Read FIFO flag.
		s2lp_status = S2LP_get_irq_flag(S2LP_IRQ_INDEX_RX_DATA_READY, &s2lp_irq_flag);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Check flag.
		if (s2lp_irq_flag != 0) {
			// Read FIFO and RSSI.
			s2lp_status = S2LP_read_fifo((sfx_u8*) rf_api_ctx.dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			s2lp_status = S2LP_get_rssi(S2LP_RSSI_TYPE_SYNC_WORD, &rf_api_ctx.dl_rssi_dbm);
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			// Stop radio.
			s2lp_status = S2LP_send_command(S2LP_COMMAND_SABORT);
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			// Clear flag.
			s2lp_status = S2LP_clear_all_irq();
			S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
			// Disable interrupt.
			rf_api_ctx.flags.field.gpio_irq_enable = 0;
			// Update state.
			rf_api_ctx.state = RF_API_STATE_READY;
		}
		break;
#endif /* BIDIRECTIONAL */
	default:
		EXIT_ERROR(RF_API_ERROR_STATE);
		break;
	}
errors:
	RETURN();
}

/*** RF API functions ***/

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t *rf_api_config) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t RF_API_close(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t RF_API_process(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t RF_API_wake_up(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_sleep(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	S2LP_modulation_t modulation = S2LP_MODULATION_NONE;
	sfx_u32 datarate_bps = 0;
	sfx_u32 deviation_hz = 0;
	// Turn radio on.
	power_status = POWER_enable(POWER_DOMAIN_RADIO, LPTIM_DELAY_MODE_SLEEP);
	POWER_stack_exit_error(RF_API_ERROR_DRIVER_POWER);
	// Exit shutdown.
	s2lp_status = S2LP_shutdown(0);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Reset chip state machine.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_SRES);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Oscillator.
	s2lp_status = S2LP_set_oscillator(S2LP_OSCILLATOR_TCXO);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	s2lp_status = S2LP_wait_for_oscillator();
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Charge pump.
	s2lp_status = S2LP_configure_charge_pump();
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Frequency.
	s2lp_status = S2LP_set_rf_frequency(radio_parameters -> frequency_hz);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Modulation parameters.
	switch (radio_parameters -> modulation) {
	case RF_API_MODULATION_NONE:
		modulation = S2LP_MODULATION_NONE;
		datarate_bps = (radio_parameters -> bit_rate_bps);
#ifdef BIDIRECTIONAL
		deviation_hz = (radio_parameters -> deviation_hz);
#endif
		// Set CW output power.
		s2lp_status = S2LP_set_rf_output_power(radio_parameters -> tx_power_dbm_eirp);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		break;
	case RF_API_MODULATION_DBPSK:
		modulation = S2LP_MODULATION_POLAR;
		datarate_bps = ((radio_parameters -> bit_rate_bps) * RF_API_SYMBOL_PROFILE_SIZE_BYTES) / (RF_API_POLAR_DATARATE_MULTIPLIER);
		deviation_hz = ((radio_parameters -> bit_rate_bps) * RF_API_SYMBOL_PROFILE_SIZE_BYTES) / (2);
		break;
	case RF_API_MODULATION_GFSK:
		modulation = S2LP_MODULATION_2GFSK_BT1;
		datarate_bps = (radio_parameters -> bit_rate_bps);
#ifdef BIDIRECTIONAL
		deviation_hz = (radio_parameters -> deviation_hz);
#endif
		break;
	default:
		EXIT_ERROR(RF_API_ERROR_MODULATION);
		break;
	}
	// Set modulation scheme.
	s2lp_status = S2LP_set_modulation(modulation);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	if (datarate_bps != 0) {
		s2lp_status = S2LP_set_datarate(datarate_bps);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	}
	if (deviation_hz != 0) {
		s2lp_status = S2LP_set_fsk_deviation(deviation_hz);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	}
	// Disable all interrupts by default.
	s2lp_status = S2LP_disable_all_irq();
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Configure specific registers.
	switch (radio_parameters -> rf_mode) {
	case RF_API_MODE_TX:
		// Enable FIFO interrupt.
		s2lp_status = S2LP_configure_irq(S2LP_IRQ_INDEX_TX_FIFO_ALMOST_EMPTY, 1);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// SMPS switching frequency.
		s2lp_status = S2LP_set_smps_frequency(RF_API_SMPS_FREQUENCY_HZ_TX);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// TX parameters.
		s2lp_status = S2LP_configure_pa();
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_set_tx_source(S2LP_TX_SOURCE_FIFO);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_set_fifo_threshold(S2LP_FIFO_THRESHOLD_TX_EMPTY, RF_API_FIFO_TX_ALMOST_EMPTY_THRESHOLD);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
		// Start latency = ramp-up.
		RF_API_LATENCY_MS[RF_API_LATENCY_SEND_START] = ((1000) / ((sfx_u32) (radio_parameters -> bit_rate_bps)));
		// Stop latency = ramp-down + half of padding bit (since IRQ is raised at the middle of the symbol).
		RF_API_LATENCY_MS[RF_API_LATENCY_SEND_STOP] = ((1500) / ((sfx_u32) (radio_parameters -> bit_rate_bps)));
#endif
		break;
#ifdef BIDIRECTIONAL
	case RF_API_MODE_RX:
		// Enable RX data ready interrupt.
		s2lp_status = S2LP_configure_irq(S2LP_IRQ_INDEX_RX_DATA_READY, 1);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// SMPS switching frequency.
		s2lp_status = S2LP_set_smps_frequency(RF_API_SMPS_FREQUENCY_HZ_RX);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// RX parameters.
		s2lp_status = S2LP_set_rx_source(S2LP_RX_SOURCE_NORMAL);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_set_rx_bandwidth(RF_API_RX_BANDWIDTH_HZ);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_set_rssi_threshold(RF_API_DOWNLINK_RSSI_THRESHOLD_DBM);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_configure_clock_recovery();
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Downlink packet structure.
		s2lp_status = S2LP_set_preamble_detector((RF_API_DL_PR_SIZE_BITS / 2), S2LP_PREAMBLE_PATTERN_1010);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_set_sync_word((uint8_t*) RF_API_DL_FT, (SIGFOX_DL_FT_SIZE_BYTES * 8));
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_set_packet_length(SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_disable_crc();
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		// Disable AFC, equalization, CS blanking and antenna switch.
		s2lp_status = S2LP_disable_afc();
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		s2lp_status = S2LP_disable_equa_cs_ant_switch();
		S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
		break;
#endif
	default:
		// Return error.
		status = RF_API_ERROR_MODE;
		goto errors;
	}
errors:
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_de_init(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	POWER_status_t power_status = POWER_SUCCESS;
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	// Turn transceiver and TCXO off.
	s2lp_status = S2LP_shutdown(1);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Turn radio off.
	power_status = POWER_disable(POWER_DOMAIN_RADIO);
	POWER_stack_exit_error(RF_API_ERROR_DRIVER_POWER);
errors:
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t *tx_data) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	sfx_u8 idx = 0;
	// Store TX data.
	rf_api_ctx.tx_bitstream_size_bytes = (tx_data -> bitstream_size_bytes);
	for (idx=0 ; idx<(rf_api_ctx.tx_bitstream_size_bytes) ; idx++) {
		rf_api_ctx.tx_bitstream[idx] = (tx_data -> bitstream)[idx];
	}
	// Enable GPIO interrupt.
	s2lp_status = S2LP_enable_nirq(S2LP_FIFO_FLAG_DIRECTION_TX, _RF_API_s2lp_gpio_irq_callback);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Init state.
	rf_api_ctx.tx_bit_idx = 0;
	rf_api_ctx.tx_byte_idx = 0;
	rf_api_ctx.state = RF_API_STATE_TX_RAMP_UP;
	rf_api_ctx.flags.all = 0;
	// Trigger TX.
	status = _RF_API_internal_process();
	CHECK_STATUS(RF_API_SUCCESS);
	// Wait for transmission to complete.
	while (rf_api_ctx.state != RF_API_STATE_READY) {
		// Wait for GPIO interrupt.
		while (rf_api_ctx.flags.field.gpio_irq_flag == 0) {
			// Enter sleep mode.
			PWR_enter_sleep_mode();
		}
		// Clear flag.
		rf_api_ctx.flags.field.gpio_irq_flag = 0;
		// Call process function.
		status = _RF_API_internal_process();
		CHECK_STATUS(RF_API_SUCCESS);
	}
errors:
	// Disable GPIO interrupt.
	S2LP_disable_nirq();
	RETURN();
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t *rx_data) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	sfx_bool dl_timeout = SFX_FALSE;
	// Enable GPIO interrupt.
	s2lp_status = S2LP_enable_nirq(S2LP_FIFO_FLAG_DIRECTION_RX, _RF_API_s2lp_gpio_irq_callback);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Reset flag.
	(rx_data -> data_received) = SFX_FALSE;
	// Init state.
	rf_api_ctx.state = RF_API_STATE_RX_START;
	rf_api_ctx.flags.all = 0;
	// Trigger RX.
	status = _RF_API_internal_process();
	CHECK_STATUS(RF_API_SUCCESS);
	// Wait for reception to complete.
	while (rf_api_ctx.state != RF_API_STATE_READY) {
		// Wait for GPIO interrupt.
		while (rf_api_ctx.flags.field.gpio_irq_flag == 0) {
			// Enter sleep mode.
			PWR_enter_sleep_mode();
			IWDG_reload();
			// Check timeout.
			mcu_api_status = MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
			MCU_API_check_status(RF_API_ERROR_DRIVER_MCU_API);
			// Exit if timeout.
			if (dl_timeout == SFX_TRUE) {
				// Stop radio.
				s2lp_status = S2LP_send_command(S2LP_COMMAND_SABORT);
				S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
				// Exit loop.
				goto errors;
			}
		}
		// Clear flag.
		rf_api_ctx.flags.field.gpio_irq_flag = 0;
		// Call process function.
		status = _RF_API_internal_process();
		CHECK_STATUS(RF_API_SUCCESS);
	}
	// Update status flag.
	(rx_data -> data_received) = SFX_TRUE;
errors:
	// Disable GPIO interrupt.
	S2LP_disable_nirq();
	RETURN();
}
#endif

#ifdef BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	sfx_u8 idx = 0;
	// Check parameters.
	if ((dl_phy_content == SFX_NULL) || (dl_rssi_dbm == SFX_NULL)) {
		EXIT_ERROR(RF_API_ERROR_NULL_PARAMETER);
	}
	if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
		EXIT_ERROR(RF_API_ERROR_BUFFER_SIZE);
	}
	// Fill data.
	for (idx=0 ; idx<dl_phy_content_size ; idx++) {
		dl_phy_content[idx] = rf_api_ctx.dl_phy_content[idx];
	}
	(*dl_rssi_dbm) = (sfx_s16) rf_api_ctx.dl_rssi_dbm;
errors:
	RETURN();
}
#endif

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	// Check parameter.
	if (latency_type >= RF_API_LATENCY_LAST) {
		EXIT_ERROR(RF_API_ERROR_LATENCY_TYPE);
	}
	// Set latency.
	(*latency_ms) = RF_API_LATENCY_MS[latency_type];
errors:
	RETURN();
}
#endif

#ifdef CERTIFICATION
/*******************************************************************/
RF_API_status_t RF_API_start_continuous_wave(void) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	// Lock PLL.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_LOCKTX);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_LOCK);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	// Start radio.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_TX);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_TX);
	S2LP_stack_exit_error(RF_API_ERROR_DRIVER_S2LP);
errors:
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
	// Local variables.
	RF_API_status_t status = RF_API_SUCCESS;
	RETURN();
}
#endif

#ifdef ERROR_CODES
/*******************************************************************/
void RF_API_error(void) {
	// Force all front-end off.
	S2LP_shutdown(1);
	POWER_disable(POWER_DOMAIN_RADIO);
}
#endif
