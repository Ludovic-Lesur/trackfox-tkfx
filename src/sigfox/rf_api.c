/*
 * rf_api.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "rf_api.h"

#include "dma.h"
#include "exti.h"
#include "iwdg.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
#include "s2lp.h"
#include "sigfox_api.h"
#include "sigfox_types.h"
#include "spi.h"

/*** RF API local macros ***/

#define RF_API_SYMBOL_PROFILE_LENGTH_BYTES		40
#define RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES	(2 * RF_API_SYMBOL_PROFILE_LENGTH_BYTES) // Size is twice to store PA and FDEV values.

#define RF_API_S2LP_FDEV_NEGATIVE				0x7F // fdev * (+1)
#define RF_API_S2LP_FDEV_POSITIVE				0x81 // fdev * (-1)
#define RF_API_S2LP_FIFO_BUFFER_FDEV_IDX		(RF_API_SYMBOL_PROFILE_LENGTH_BYTES / 2) // Index where deviation is performed to invert phase.

#define RF_API_UPLINK_DATARATE_100BPS			S2LP_DATARATE_500BPS // 500*8 = 4kHz / 40 samples = 100bps.
#define RF_API_UPLINK_DEVIATION_100BPS			S2LP_FDEV_2KHZ // 1 / (2 * Delta_f) = 1 / 4kHz.

// Ramp profile table is written for ramp-down direction (reverse table for ramp up).
static const uint8_t RF_API_RAMP_AMPLITUDE_PROFILE_14DBM[RF_API_SYMBOL_PROFILE_LENGTH_BYTES] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 13, 15, 17, 20, 22, 24, 27, 30, 34, 39, 45, 54, 80, 120, 220};
// Ampltude profile table for bit 0 transmission.
static const uint8_t RF_API_BIT0_AMPLITUDE_PROFILE_14DBM[RF_API_SYMBOL_PROFILE_LENGTH_BYTES] = {1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 6, 8, 11, 15, 20, 24, 30, 39, 54, 220, 220, 54, 39, 30, 24, 20, 15, 11, 8, 6, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1};

#define RF_API_DOWNLINK_DATARATE				S2LP_DATARATE_600BPS
#define RF_API_DOWNLINK_DEVIATION				S2LP_FDEV_800HZ
#define RF_API_DOWNLINK_RX_BANDWIDTH			S2LP_RXBW_2KHZ1

#define RF_API_DOWNLINK_FRAME_LENGTH_BYTES		15
#define RF_API_DOWNLINK_TIMEOUT_SECONDS			25
#define RF_API_DOWNLINK_PREAMBLE_LENGTH_BITS	32 // 0xAAAAAAAA.
#define RF_API_DOWNLINK_SYNC_WORD_LENGTH_BITS	16 // 0xB227.
static const uint8_t RF_API_DOWNLINK_SYNC_WORD[(RF_API_DOWNLINK_SYNC_WORD_LENGTH_BITS / 8)] = {0xB2, 0x27};

/*** RF API local structures ***/

typedef struct {
	uint8_t s2lp_fifo_buffer[RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES];
	volatile uint8_t s2lp_irq_flag;
	int8_t cw_output_power;
} RF_api_context_t;

/*** RF API local global variables ***/

static RF_api_context_t rf_api_ctx = {
	.s2lp_irq_flag = 0,
	.cw_output_power = 14,
};

/*** RF API functions ***/

/*!******************************************************************
 * \fn sfx_u8 RF_API_init(sfx_rf_mode_t rf_mode)
 * \brief Init and configure Radio link in RX/TX
 *
 * [RX Configuration]
 * To receive Sigfox Frame on your device, program the following:
 *  - Preamble  : 0xAAAAAAAAA
 *  - Sync Word : 0xB227
 *  - Packet of the Sigfox frame is 15 bytes length.
 *
 * \param[in] sfx_rf_mode_t rf_mode         Init Radio link in Tx or RX
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:             No error
 * \retval RF_ERR_API_INIT:          Init Radio link error
 *******************************************************************/
sfx_u8 RF_API_init(sfx_rf_mode_t rf_mode) {
	// Local variables.
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	// Clear watchdog.
	IWDG_reload();
	// Turn transceiver on.
	spi_status = SPI1_power_on();
	if (spi_status != SPI_SUCCESS) goto errors;
	s2lp_status = S2LP_tcxo(1);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_shutdown(0);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// TX/RX common init.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_SRES);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_send_command(S2LP_COMMAND_STANDBY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_STANDBY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_set_oscillator(S2LP_OSCILLATOR_TCXO);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_configure_charge_pump();
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// Dedicated configurations.
	switch (rf_mode) {
	case SFX_RF_MODE_TX:
		// Configure GPIO.
		GPIO_configure(&GPIO_S2LP_GPIO0, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
		EXTI_configure_gpio(&GPIO_S2LP_GPIO0, EXTI_TRIGGER_RISING_EDGE);
		// Uplink.
		s2lp_status = S2LP_configure_smps(S2LP_SMPS_TX);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_configure_pa();
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_modulation(S2LP_MODULATION_POLAR);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_tx_source(S2LP_TX_SOURCE_FIFO);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_bitrate(RF_API_UPLINK_DATARATE_100BPS);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_fsk_deviation(RF_API_UPLINK_DEVIATION_100BPS);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_fifo_threshold(S2LP_FIFO_THRESHOLD_TX_EMPTY, (S2LP_FIFO_SIZE_BYTES - RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES));
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_configure_gpio(0, S2LP_GPIO_MODE_OUT_LOW_POWER, S2LP_GPIO_OUTPUT_FUNCTION_FIFO_EMPTY, 0);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		break;
	case SFX_RF_MODE_RX:
		// Configure GPIO.
		GPIO_configure(&GPIO_S2LP_GPIO0, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_UP);
		EXTI_configure_gpio(&GPIO_S2LP_GPIO0, EXTI_TRIGGER_FALLING_EDGE);
		// Downlink.
		s2lp_status = S2LP_configure_smps(S2LP_SMPS_RX);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_modulation(S2LP_MODULATION_2GFSK_BT1);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_fsk_deviation(RF_API_DOWNLINK_DEVIATION);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_bitrate(RF_API_DOWNLINK_DATARATE);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_rx_bandwidth(RF_API_DOWNLINK_RX_BANDWIDTH);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_configure_gpio(0, S2LP_GPIO_MODE_OUT_LOW_POWER, S2LP_GPIO_OUTPUT_FUNCTION_NIRQ, 1);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_configure_irq(S2LP_IRQ_INDEX_RX_DATA_READY, 1);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		// Downlink packet structure.
		s2lp_status = S2LP_set_preamble_detector((RF_API_DOWNLINK_PREAMBLE_LENGTH_BITS / 2), S2LP_PREAMBLE_PATTERN_1010);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_sync_word((uint8_t*) RF_API_DOWNLINK_SYNC_WORD, RF_API_DOWNLINK_SYNC_WORD_LENGTH_BITS);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_set_packet_length(RF_API_DOWNLINK_FRAME_LENGTH_BYTES);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_disable_crc();
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		// Disable CS blanking, equalization and antenna switching.
		s2lp_status = S2LP_disable_equa_cs_ant_switch();
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		// FIFO.
		s2lp_status = S2LP_set_rx_source(S2LP_RX_SOURCE_NORMAL);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		break;
	default:
		goto errors;
		break;
	}
	return SFX_ERR_NONE;
errors:
	SPI1_power_off();
	return RF_ERR_API_INIT;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_stop(void)
 * \brief Close Radio link
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:              No error
 * \retval RF_ERR_API_STOP:           Close Radio link error
 *******************************************************************/
sfx_u8 RF_API_stop(void) {
	// Local variables.
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	// Turn transceiver and TCXO off.
	s2lp_status = S2LP_shutdown(1);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_tcxo(0);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	GPIO_configure(&GPIO_S2LP_GPIO0, GPIO_MODE_OUTPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	SPI1_power_off();
	return SFX_ERR_NONE;
errors:
	SPI1_power_off();
	return RF_ERR_API_STOP;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_send(sfx_u8 *stream, sfx_modulation_type_t type, sfx_u8 size)
 * \brief BPSK Modulation of data stream
 * (from synchro bit field to CRC)
 *
 * NOTE : during this function, the voltage_tx needs to be retrieved and stored in
 *        a variable to be returned into the MCU_API_get_voltage_and_temperature or
 *        MCU_API_get_voltage functions.
 *
 * \param[in] sfx_u8 *stream                Complete stream to modulate
 * \param[in]sfx_modulation_type_t          Type of the modulation ( enum with baudrate and modulation information)
 * \param[in] sfx_u8 size                   Length of stream
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                    No error
 * \retval RF_ERR_API_SEND:                 Send data stream error
 *******************************************************************/
sfx_u8 RF_API_send(sfx_u8 *stream, sfx_modulation_type_t type, sfx_u8 size) {
	// Local variables.
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	uint8_t stream_byte_idx = 0;
	uint8_t stream_bit_idx = 0;
	uint8_t s2lp_fifo_sample_idx = 0;
	uint8_t s2lp_fdev = RF_API_S2LP_FDEV_NEGATIVE; // Effective deviation.
	// Go to ready state.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// First ramp-up.
	for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_SYMBOL_PROFILE_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
		rf_api_ctx.s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx)] = 0; // No deviation.
		rf_api_ctx.s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx) + 1] = RF_API_RAMP_AMPLITUDE_PROFILE_14DBM[RF_API_SYMBOL_PROFILE_LENGTH_BYTES - s2lp_fifo_sample_idx - 1]; // PA output power for ramp-up.
	}
	// Transfer ramp-up buffer to S2LP FIFO.
	s2lp_status = S2LP_write_fifo(rf_api_ctx.s2lp_fifo_buffer, RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// Enable external GPIO interrupt.
	EXTI_clear_all_flags();
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI_4_15);
	// Start radio
	s2lp_status = S2LP_send_command(S2LP_COMMAND_TX);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// Byte loop.
	for (stream_byte_idx=0 ; stream_byte_idx<size ; stream_byte_idx++) {
		// Bit loop.
		for (stream_bit_idx=0 ; stream_bit_idx<8 ; stream_bit_idx++) {
			if ((stream[stream_byte_idx] & (0b1 << (7-stream_bit_idx))) == 0) {
				// Phase shift and amplitude shaping required.
				s2lp_fdev = (s2lp_fdev == RF_API_S2LP_FDEV_NEGATIVE) ? RF_API_S2LP_FDEV_POSITIVE : RF_API_S2LP_FDEV_NEGATIVE; // Toggle deviation.
				for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_SYMBOL_PROFILE_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
					rf_api_ctx.s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx)] = (s2lp_fifo_sample_idx == RF_API_S2LP_FIFO_BUFFER_FDEV_IDX) ? s2lp_fdev : 0; // Deviation.
					rf_api_ctx.s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx) + 1] = RF_API_BIT0_AMPLITUDE_PROFILE_14DBM[s2lp_fifo_sample_idx]; // PA output power.
				}
			}
			else {
				// Constant CW.
				for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_SYMBOL_PROFILE_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
					rf_api_ctx.s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx)] = 0; // No deviation.
					rf_api_ctx.s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx) + 1] = 1; // Constant PA output power.
				}
			}
			// Enter stop and wait for S2LP interrupt to transfer next bit buffer.
			rf_api_ctx.s2lp_irq_flag = 0;
			while (rf_api_ctx.s2lp_irq_flag == 0) {
				PWR_enter_stop_mode();
			}
			s2lp_status = S2LP_write_fifo(rf_api_ctx.s2lp_fifo_buffer, RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES);
			if (s2lp_status != S2LP_SUCCESS) goto errors;
		}
	}
	// Last ramp down.
	for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_SYMBOL_PROFILE_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
		rf_api_ctx.s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx)] = 0; // FDEV.
		rf_api_ctx.s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx) + 1] = RF_API_RAMP_AMPLITUDE_PROFILE_14DBM[s2lp_fifo_sample_idx]; // PA output power for ramp-down.
	}
	// Enter stop and wait for S2LP interrupt to transfer ramp-down buffer.
	PWR_enter_stop_mode();
	s2lp_status = S2LP_write_fifo(rf_api_ctx.s2lp_fifo_buffer, RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// Padding bit to ensure ramp-down is completely transmitted.
	for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
		rf_api_ctx.s2lp_fifo_buffer[s2lp_fifo_sample_idx] = 0;
	}
	// Enter stop and wait for S2LP interrupt to transfer padding buffer.
	PWR_enter_stop_mode();
	s2lp_status = S2LP_write_fifo(rf_api_ctx.s2lp_fifo_buffer, RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// Enter stop and wait for S2LP interrupt.
	PWR_enter_stop_mode();
	// Disable external GPIO interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_EXTI_4_15);
	// Stop radio.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_SABORT);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_send_command(S2LP_COMMAND_STANDBY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_STANDBY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	return SFX_ERR_NONE;
errors:
	return RF_ERR_API_SEND;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_start_continuous_transmission (sfx_modulation_type_t type)
 * \brief Generate a signal with modulation type. All the configuration ( Init of the RF and Frequency have already been executed
 *        when this function is called.
 *
 * \param[in] sfx_modulation_type_t         Type of the modulation ( enum with baudrate and modulation information is contained in sigfox_api.h)
 *
 * \retval SFX_ERR_NONE:                                 No error
 * \retval RF_ERR_API_START_CONTINUOUS_TRANSMISSION:     Continuous Transmission Start error
 *******************************************************************/
sfx_u8 RF_API_start_continuous_transmission(sfx_modulation_type_t type) {
	// Local variables.
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	// Disable modulation.
	s2lp_status = S2LP_set_modulation(S2LP_MODULATION_NONE);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_set_rf_output_power(rf_api_ctx.cw_output_power);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// Start radio.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_send_command(S2LP_COMMAND_TX);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	return SFX_ERR_NONE;
errors:
	return RF_ERR_API_START_CONTINUOUS_TRANSMISSION;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_stop_continuous_transmission (void)
 * \brief Stop the current continuous transmisssion
 *
 * \retval SFX_ERR_NONE:                                 No error
 * \retval RF_ERR_API_STOP_CONTINUOUS_TRANSMISSION:      Continuous Transmission Stop error
 *******************************************************************/
sfx_u8 RF_API_stop_continuous_transmission(void) {
	// Local variables.
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	// Stop radio.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_SABORT);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	return SFX_ERR_NONE;
errors:
	return RF_ERR_API_STOP_CONTINUOUS_TRANSMISSION;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_change_frequency(sfx_u32 frequency)
 * \brief Change synthesizer carrier frequency
 *
 * \param[in] sfx_u32 frequency             Frequency in Hz to program in the radio chipset
 * \param[out] none
 *
 * \retval SFX_ERR_NONE:                    No error
 * \retval RF_ERR_API_CHANGE_FREQ:          Change frequency error
 *******************************************************************/
sfx_u8 RF_API_change_frequency(sfx_u32 frequency) {
	// Local variables.
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	// Set frequency.
	s2lp_status = S2LP_set_rf_frequency(frequency);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	return SFX_ERR_NONE;
errors:
	return RF_ERR_API_CHANGE_FREQ;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_wait_frame(sfx_u8 *frame, sfx_s16 *rssi, sfx_rx_state_enum_t * state)
 * \brief Get all GFSK frames received in Rx buffer, structure of
 * frame is : Synchro bit + Synchro frame + 15 Bytes.<BR> This function must
 * be blocking state since data is received or timer of 25 s has elapsed.
 *
 * - If received buffer, function returns SFX_ERR_NONE then the
 *   library will try to decode frame. If the frame is not correct, the
 *   library will recall RF_API_wait_frame.
 *
 * - If 25 seconds timer has elapsed, function returns into the state the timeout enum code.
 *   and then library will stop receive frame phase.
 *
 * \param[in] none
 * \param[out] sfx_s8 *frame                  Receive buffer
 * \param[out] sfx_s16 *rssi                  Chipset RSSI
 * Warning: This is the 'raw' RSSI value. Do not add 100 as made
 * in Library versions 1.x.x
 * Resolution: 1 LSB = 1 dBm
 *
 * \param[out] sfx_rx_state_enum_t state      Indicate the final state of the reception. Value can be DL_TIMEOUT or DL_PASSED
 *                                            if a frame has been received, as defined in sigfox_api.h file.
 *
 * \retval SFX_ERR_NONE:                      No error
 *******************************************************************/
sfx_u8 RF_API_wait_frame(sfx_u8 *frame, sfx_s16 *rssi, sfx_rx_state_enum_t * state) {
	// Local variables.
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	RTC_status_t rtc_status = RTC_SUCCESS;
	int16_t rssi_dbm = 0;
	uint32_t remaining_delay = RF_API_DOWNLINK_TIMEOUT_SECONDS;
	uint32_t sub_delay = 0;
	// Init state.
	(*state) = DL_TIMEOUT;
	// Go to ready state.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_send_command(S2LP_COMMAND_FLUSHRXFIFO);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_clear_irq_flags();
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	rf_api_ctx.s2lp_irq_flag = 0;
	// Start radio.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_RX);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	// Enable external GPIO.
	EXTI_clear_all_flags();
	NVIC_enable_interrupt(NVIC_INTERRUPT_EXTI_4_15);
	// Clear watchdog.
	IWDG_reload();
	// Enter stop mode until GPIO interrupt or RTC wake-up.
	while ((remaining_delay > 0) && (rf_api_ctx.s2lp_irq_flag == 0)) {
		// Compute sub-delay.
		sub_delay = (remaining_delay > IWDG_REFRESH_PERIOD_SECONDS) ? (IWDG_REFRESH_PERIOD_SECONDS) : (remaining_delay);
		remaining_delay -= sub_delay;
		// Restart wake-up timer.
		rtc_status = RTC_stop_wakeup_timer();
		if (rtc_status != RTC_SUCCESS) goto errors;
		rtc_status = RTC_start_wakeup_timer(sub_delay);
		if (rtc_status != RTC_SUCCESS) goto errors;
		// Enter stop mode.
		PWR_enter_stop_mode();
		// Wake-up: clear watchdog and flags.
		IWDG_reload();
		RTC_clear_wakeup_timer_flag();
		EXTI_clear_all_flags();
	}
	// Wake-up: disable interrupts.
	rtc_status = RTC_stop_wakeup_timer();
	if (rtc_status != RTC_SUCCESS) goto errors;
	NVIC_disable_interrupt(NVIC_INTERRUPT_EXTI_4_15);
	// Check flag.
	if (rf_api_ctx.s2lp_irq_flag != 0) {
		// Downlink frame received.
		s2lp_status = S2LP_read_fifo(frame, RF_API_DOWNLINK_FRAME_LENGTH_BYTES);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		s2lp_status = S2LP_get_rssi(S2LP_RSSI_TYPE_SYNC_WORD, &rssi_dbm);
		if (s2lp_status != S2LP_SUCCESS) goto errors;
		// Update status.
		(*state) = DL_PASSED;
		(*rssi) = (sfx_s16) rssi_dbm;
	}
	// Stop radio.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_SABORT);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_READY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_send_command(S2LP_COMMAND_STANDBY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_STANDBY);
	if (s2lp_status != S2LP_SUCCESS) goto errors;
	return SFX_ERR_NONE;
errors:
	RTC_stop_wakeup_timer();
	return RF_ERR_API_WAIT_FRAME;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_wait_for_clear_channel (sfx_u8 cs_min, sfx_s8 cs_threshold, sfx_rx_state_enum_t * state);
 * \brief This function is used in ARIB standard for the Listen Before Talk
 *        feature. It listens on a specific frequency band initialized through the RF_API_init(), during a sliding window set
 *        in the MCU_API_timer_start_carrier_sense().
 *        If the channel is clear during the minimum carrier sense
 *        value (cs_min), under the limit of the cs_threshold,
 *        the functions returns with SFX_ERR_NONE (transmission
 *        allowed). Otherwise it continues to listen to the channel till the expiration of the
 *        carrier sense maximum window and then updates the state ( with timeout enum ).
 *
 * \param[in] none
 * \param[out] sfx_u8 cs_min                  Minimum Carrier Sense time in ms.
 * \param[out] sfx_s8 cs_threshold            Power threshold limit to declare the channel clear.
 *                                            i.e : cs_threshold value -80dBm in Japan / -65dBm in Korea
 * \param[out] sfx_rx_state_enum_t state      Indicate the final state of the carrier sense. Value can be DL_TIMEOUT or PASSED
 *                                            as per defined in sigfox_api.h file.
 *
 * \retval SFX_ERR_NONE:                      No error
 *******************************************************************/
sfx_u8 RF_API_wait_for_clear_channel(sfx_u8 cs_min, sfx_s8 cs_threshold, sfx_rx_state_enum_t * state) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_get_version(sfx_u8 **version, sfx_u8 *size)
 * \brief Returns current RF API version
 *
 * \param[out] sfx_u8 **version                 Pointer to Byte array (ASCII format) containing library version
 * \param[out] sfx_u8 *size                     Size of the byte array pointed by *version
 *
 * \retval SFX_ERR_NONE:                No error
 * \retval RF_ERR_API_GET_VERSION:      Get Version error
 *******************************************************************/
sfx_u8 RF_API_get_version(sfx_u8 **version, sfx_u8 *size) {
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_set_irq_flag(void)
 * \brief Set S2LP IRQ flag.
 *
 * \param[in] none
 * \param[out] none
 *
 * \retval none
 *******************************************************************/
void RF_API_set_irq_flag(void) {
	rf_api_ctx.s2lp_irq_flag = 1;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_set_cw_output_power(void)
 * \brief Store TX output power for CW mode.
 *
 * \param[in] tx_output_power: RF output power in dBm.
 * \param[out] none
 *
 * \retval none
 *******************************************************************/
void RF_API_set_cw_output_power(int8_t tx_output_power) {
	rf_api_ctx.cw_output_power = tx_output_power;
}
