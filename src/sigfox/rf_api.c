/*
 * rf_api.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "rf_api.h"

#include "dma.h"
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "mapping.h"
#include "mode.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "s2lp.h"
#include "sigfox_api.h"
#include "sigfox_types.h"
#include "spi.h"
#include "usart.h"

#include "exti_reg.h"
#include "s2lp_reg.h"

/*** RF API local macros ***/

// Uplink parameters.
#define RF_API_SYMBOL_PROFILE_LENGTH_BYTES		40
#define RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES	(2 * RF_API_SYMBOL_PROFILE_LENGTH_BYTES) // Size is twice to store PA and FDEV values.

#define RF_API_S2LP_FDEV_NEGATIVE				0x7F // fdev * (+1)
#define RF_API_S2LP_FDEV_POSITIVE				0x81 // fdev * (-1)
#define RF_API_S2LP_FIFO_BUFFER_FDEV_IDX		(RF_API_SYMBOL_PROFILE_LENGTH_BYTES / 2) // Index where deviation is performed to invert phase.

#define RF_API_ETSI_UPLINK_OUTPUT_POWER_DBM		14
#define RF_API_ESTI_UPLINK_DATARATE				S2LP_DATARATE_500BPS // 500*8 = 4kHz / 40 samples = 100bps.
#define RF_API_ETSI_UPLINK_DEVIATION			S2LP_FDEV_2KHZ // 1 / (2 * Delta_f) = 1 / 4kHz.

// Ramp profile table is written for ramp-down direction (reverse table for ramp up).
static const unsigned char rf_api_etsi_ramp_amplitude_profile[RF_API_SYMBOL_PROFILE_LENGTH_BYTES] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 9, 11, 13, 15, 17, 20, 22, 24, 27, 30, 34, 39, 45, 54, 80, 120, 220};
// Ampltude profile table for bit 0 transmission.
static const unsigned char rf_api_etsi_bit0_amplitude_profile[RF_API_SYMBOL_PROFILE_LENGTH_BYTES] = {1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 6, 8, 11, 15, 20, 24, 30, 39, 54, 220, 220, 54, 39, 30, 24, 20, 15, 11, 8, 6, 4, 3, 2, 2, 1, 1, 1, 1, 1, 1};

// Downlink parameters.
#define RF_API_DOWNLINK_FRAME_LENGTH_BYTES		15
#define RF_API_DOWNLINK_TIMEOUT_SECONDS			25
#define RF_API_ESTI_DOWNLINK_DATARATE			S2LP_DATARATE_600BPS
#define RF_API_ETSI_DOWNLINK_DEVIATION			S2LP_FDEV_800HZ

/*** RF API local global variables ***/

static unsigned char rf_api_s2lp_fifo_buffer[RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES];

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
	// Init required peripherals.
	DMA1_InitChannel3();
	SPI1_Init();
	// Turn TCXO and transceiver on.
	RCC_Tcxo(1);
	SPI1_PowerOn();
	// TX/RX common init.
	S2LP_SendCommand(S2LP_CMD_STANDBY);
	S2LP_WaitForStateSwitch(S2LP_STATE_STANDBY);
	S2LP_SetOscillator(S2LP_OSCILLATOR_TCXO);
	S2LP_ConfigureSmps();
	S2LP_ConfigureChargePump();
	// Dedicated configurations.
	switch (rf_mode) {
	case SFX_RF_MODE_TX:
		// Uplink.
		S2LP_ConfigurePa();
		S2LP_SetModulation(S2LP_MODULATION_POLAR);
		S2LP_SetTxSource(S2LP_TX_SOURCE_FIFO);
		S2LP_SetFskDeviation(RF_API_ETSI_UPLINK_DEVIATION);
		S2LP_SetBitRate(RF_API_ESTI_UPLINK_DATARATE);
		S2LP_SetFifoThreshold(S2LP_FIFO_THRESHOLD_TX_EMPTY, (S2LP_FIFO_SIZE_BYTES - RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES));
		S2LP_ConfigureGpio(0, S2LP_GPIO_MODE_OUT_LOW_POWER, S2LP_GPIO_OUTPUT_FUNCTION_FIFO_EMPTY, 0);
		break;
	case SFX_RF_MODE_RX:
		// Downlink.
		S2LP_SetModulation(S2LP_MODULATION_2GFSK_BT1);
		S2LP_SetFskDeviation(RF_API_ETSI_DOWNLINK_DEVIATION);
		S2LP_SetBitRate(RF_API_ESTI_DOWNLINK_DATARATE);
		break;
	default:
		// Unknwon mode.
		break;
	}
	return SFX_ERR_NONE;
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
	// Turn transceiver and TCXO off.
	SPI1_PowerOff();
	RCC_Tcxo(0);
	// Turn peripherals off.
	DMA1_Disable();
	SPI1_Disable();
	return SFX_ERR_NONE;
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
	unsigned char stream_byte_idx = 0;
	unsigned char stream_bit_idx = 0;
	unsigned char s2lp_fifo_sample_idx = 0;
	unsigned char s2lp_fdev = RF_API_S2LP_FDEV_NEGATIVE; // Effective deviation.
	// Go to ready state.
	S2LP_SendCommand(S2LP_CMD_READY);
	S2LP_WaitForStateSwitch(S2LP_STATE_READY);
	// First ramp-up.
	for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_SYMBOL_PROFILE_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
		rf_api_s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx)] = 0; // No deviation.
		rf_api_s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx) + 1] = rf_api_etsi_ramp_amplitude_profile[RF_API_SYMBOL_PROFILE_LENGTH_BYTES - s2lp_fifo_sample_idx - 1]; // PA output power for ramp-up.
	}
	// Transfer ramp-up buffer to S2LP FIFO.
	S2LP_WriteFifo(rf_api_s2lp_fifo_buffer, RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES);
	// Enable external GPIO interrupt.
	EXTI_ClearAllFlags();
	NVIC_EnableInterrupt(IT_EXTI_4_15);
	// Start radio
	S2LP_SendCommand(S2LP_CMD_TX);
	// Byte loop.
	for (stream_byte_idx=0 ; stream_byte_idx<size ; stream_byte_idx++) {
		// Bit loop.
		for (stream_bit_idx=0 ; stream_bit_idx<8 ; stream_bit_idx++) {
			if ((stream[stream_byte_idx] & (0b1 << (7-stream_bit_idx))) == 0) {
				// Phase shift and amplitude shaping required.
				s2lp_fdev = (s2lp_fdev == RF_API_S2LP_FDEV_NEGATIVE) ? RF_API_S2LP_FDEV_POSITIVE : RF_API_S2LP_FDEV_NEGATIVE; // Toggle deviation.
				for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_SYMBOL_PROFILE_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
					rf_api_s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx)] = (s2lp_fifo_sample_idx == RF_API_S2LP_FIFO_BUFFER_FDEV_IDX) ? s2lp_fdev : 0; // Deviation.
					rf_api_s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx) + 1] = rf_api_etsi_bit0_amplitude_profile[s2lp_fifo_sample_idx]; // PA output power.
				}
			}
			else {
				// Constant CW.
				for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_SYMBOL_PROFILE_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
					rf_api_s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx)] = 0; // No deviation.
					rf_api_s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx) + 1] = 1; // Constant PA output power.
				}
			}
			// Enter stop and wait for S2LP interrupt to transfer next bit buffer.
			PWR_EnterStopMode();
			S2LP_WriteFifo(rf_api_s2lp_fifo_buffer, RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES);
		}
	}
	// Last ramp down.
	for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_SYMBOL_PROFILE_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
		rf_api_s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx)] = 0; // FDEV.
		rf_api_s2lp_fifo_buffer[(2 * s2lp_fifo_sample_idx) + 1] = rf_api_etsi_ramp_amplitude_profile[s2lp_fifo_sample_idx]; // PA output power for ramp-down.
	}
	// Enter stop and wait for S2LP interrupt to transfer ramp-down buffer.
	PWR_EnterStopMode();
	S2LP_WriteFifo(rf_api_s2lp_fifo_buffer, RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES);
	// Padding bit to ensure ramp-down is completely transmitted.
	for (s2lp_fifo_sample_idx=0 ; s2lp_fifo_sample_idx<RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES ; s2lp_fifo_sample_idx++) {
		rf_api_s2lp_fifo_buffer[s2lp_fifo_sample_idx] = 0;
	}
	// Enter stop and wait for S2LP interrupt to transfer padding buffer.
	PWR_EnterStopMode();
	S2LP_WriteFifo(rf_api_s2lp_fifo_buffer, RF_API_S2LP_FIFO_BUFFER_LENGTH_BYTES);
	// Enter stop and wait for S2LP interrupt.
	PWR_EnterStopMode();
	// Disable external GPIO interrupt.
	NVIC_DisableInterrupt(IT_EXTI_4_15);
	// Stop radio.
	S2LP_SendCommand(S2LP_CMD_SABORT);
	S2LP_WaitForStateSwitch(S2LP_STATE_READY);
	S2LP_SendCommand(S2LP_CMD_STANDBY);
	S2LP_WaitForStateSwitch(S2LP_STATE_STANDBY);
	// Return.
	return SFX_ERR_NONE;
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
sfx_u8 RF_API_start_continuous_transmission (sfx_modulation_type_t type) {
	// Disable modulation.
	S2LP_SetModulation(S2LP_MODULATION_NONE);
	// Start radio.
	S2LP_SendCommand(S2LP_CMD_READY);
	S2LP_WaitForStateSwitch(S2LP_STATE_READY);
	S2LP_SendCommand(S2LP_CMD_TX);
	// Return.
	return SFX_ERR_NONE;
}

/*!******************************************************************
 * \fn sfx_u8 RF_API_stop_continuous_transmission (void)
 * \brief Stop the current continuous transmisssion
 *
 * \retval SFX_ERR_NONE:                                 No error
 * \retval RF_ERR_API_STOP_CONTINUOUS_TRANSMISSION:      Continuous Transmission Stop error
 *******************************************************************/
sfx_u8 RF_API_stop_continuous_transmission (void) {
	// Stop radio.
	S2LP_SendCommand(S2LP_CMD_SABORT);
	S2LP_WaitForStateSwitch(S2LP_STATE_READY);
	// Return.
	return SFX_ERR_NONE;
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
	// Set frequency.
	S2LP_SetRfFrequency(frequency);
	return SFX_ERR_NONE;
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
	// TBD.
	return SFX_ERR_NONE;
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
