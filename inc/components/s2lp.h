/*
 * s2lp.h
 *
 *  Created on: 12 oct. 2019
 *      Author: Ludo
 */

#ifndef S2LP_H
#define S2LP_H

#include "s2lp_reg.h"

/*** S2LP structures ***/

// Chip state.
typedef enum {
	S2LP_STATE_READY = 0x00,
	S2LP_STATE_SLEEP_A,
	S2LP_STATE_STANDBY,
	S2LP_STATE_SLEEP_B,
	S2LP_STATE_LOCK = 0x0C,
	S2LP_STATE_RX = 0x30,
	S2LP_STATE_SYNTH_SETUP = 0x50,
	S2LP_STATE_TX = 0x5C
} S2LP_State;

// Modulations.
typedef enum {
	S2LP_MODULATION_2FSK = 0x00,
	S2LP_MODULATION_4FSK,
	S2LP_MODULATION_2GFSK_BT1,
	S2LP_MODULATION_4GFSK_BT1,
	S2LP_MODULATION_ASK_OOK,
	S2LP_MODULATION_POLAR,
	S2LP_MODULATION_NONE,
	S2LP_MODULATION_2GFSK_BT05 = 0x0A,
	S2LP_MODULATION_4GFSK_BT05
} S2LP_Modulation;

// GPIOs modes.
typedef enum {
	S2LP_GPIO_MODE_IN = 0x01,
	S2LP_GPIO_MODE_OUT_LOW_POWER,
	S2LP_GPIO_MODE_OUT_HIGH_POWER,
} S2LP_GPIO_Mode;

// GPIOs output functions.
typedef enum {
	S2LP_GPIO_OUTPUT_FUNCTION_NIRQ = 0x00,
	S2LP_GPIO_OUTPUT_FUNCTION_NPOR,
	S2LP_GPIO_OUTPUT_FUNCTION_WUT,
	S2LP_GPIO_OUTPUT_FUNCTION_LOW_BATT,
	S2LP_GPIO_OUTPUT_FUNCTION_TX_DATA_CLOCK,
	S2LP_GPIO_OUTPUT_FUNCTION_TX_STATE,
	S2LP_GPIO_OUTPUT_FUNCTION_FIFO_EMPTY,
	S2LP_GPIO_OUTPUT_FUNCTION_FIFO_FULL,
	S2LP_GPIO_OUTPUT_FUNCTION_RX_DATA,
	S2LP_GPIO_OUTPUT_FUNCTION_RX_CLOCK,
	S2LP_GPIO_OUTPUT_FUNCTION_RX_STATE,
	S2LP_GPIO_OUTPUT_FUNCTION_SLEEP_STANDBY,
	S2LP_GPIO_OUTPUT_FUNCTION_STANDBY,
	S2LP_GPIO_OUTPUT_FUNCTION_ANTENNA,
	S2LP_GPIO_OUTPUT_FUNCTION_PREAMBLE,
	S2LP_GPIO_OUTPUT_FUNCTION_SYNC_WORD,
	S2LP_GPIO_OUTPUT_FUNCTION_RSSI,
	S2LP_GPIO_OUTPUT_FUNCTION_TX_RX = 0x12,
	S2LP_GPIO_OUTPUT_FUNCTION_VDD,
	S2LP_GPIO_OUTPUT_FUNCTION_GND,
	S2LP_GPIO_OUTPUT_FUNCTION_SMPS,
	S2LP_GPIO_OUTPUT_FUNCTION_SLEEP,
	S2LP_GPIO_OUTPUT_FUNCTION_READY,
	S2LP_GPIO_OUTPUT_FUNCTION_LOCK,
	S2LP_GPIO_OUTPUT_FUNCTION_LOCK_DETECTOR,
	S2LP_GPIO_OUTPUT_FUNCTION_TX_DATA_OOK,
	S2LP_GPIO_OUTPUT_FUNCTION_READY2,
	S2LP_GPIO_OUTPUT_FUNCTION_PM,
	S2LP_GPIO_OUTPUT_FUNCTION_VCO,
	S2LP_GPIO_OUTPUT_FUNCTION_SYNTH,
} S2LP_GPIO_OutputFunction;

// GPIO input functions.
typedef enum {
	S2LP_GPIO_INPUT_FUNCTION_TX_COMMAND,
	S2LP_GPIO_INPUT_FUNCTION_RX_COMMAND,
	S2LP_GPIO_INPUT_FUNCTION_TX_DATA,
	S2LP_GPIO_INPUT_FUNCTION_WAKE_UP,
	S2LP_GPIO_INPUT_FUNCTION_EXT_CLOCK,
} S2LP_GPIO_InputFunction;

/*** S2LP functions ***/

void S2LP_Init(void);
void S2LP_DisableGpio(void);
void S2LP_SendCommand(S2LP_Command command);
void S2LP_SetModulation(S2LP_Modulation modulation);
void S2LP_SetRfFrequency(unsigned int rf_frequency_hz);
void S2LP_SetFskDeviation(unsigned short fsk_deviation_hz);
void S2LP_SetBitRate(unsigned int bit_rate_bps);
void S2LP_ConfigureGpio(unsigned char gpio_number, S2LP_GPIO_Mode gpio_mode, unsigned char gpio_function);
unsigned short S2LP_GetIrqFlags(void);
void S2LP_SetRfOutputPower(signed char rf_output_power_dbm);
void S2LP_SetRxBandwidth(unsigned char rxbw_mantissa, unsigned char rxbw_exponent);
void S2LP_SetPreambleDetector(unsigned char preamble_length_bytes, unsigned char preamble_polarity);
void S2LP_SetSyncWord(unsigned char* sync_word, unsigned char sync_word_length_bytes);
void S2LP_SetDataLength(unsigned char data_length_bytes);
unsigned char S2LP_GetRssi(void);
void S2LP_ReadRxFifo(unsigned char* rx_data, unsigned char rx_data_length_bytes);

#endif /* S2LP_H */
