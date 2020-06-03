/*
 * s2lp.h
 *
 *  Created on: 12 oct. 2019
 *      Author: Ludo
 */

#ifndef S2LP_H
#define S2LP_H

#include "s2lp_reg.h"

/*** S2LP macros ***/

#define S2LP_FIFO_SIZE_BYTES	128

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

// oscillator.
typedef enum {
	S2LP_OSCILLATOR_QUARTZ = 0x00,
	S2LP_OSCILLATOR_TCXO
} S2LP_Oscillator;

// Modulations.
typedef enum {
	S2LP_MODULATION_2FSK = 0x00,
	S2LP_MODULATION_4FSK,
	S2LP_MODULATION_2GFSK_BT1,
	S2LP_MODULATION_4GFSK_BT1,
	S2LP_MODULATION_ASK_OOK = 0x05,
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

typedef enum {
	S2LP_FIFO_THRESHOLD_RX_FULL = 0x3C,
	S2LP_FIFO_THRESHOLD_RX_EMPTY,
	S2LP_FIFO_THRESHOLD_TX_FULL,
	S2LP_FIFO_THRESHOLD_TX_EMPTY,
} S2LP_FifoThreshold;

typedef enum {
	S2LP_TX_SOURCE_NORMAL = 0x00,
	S2LP_TX_SOURCE_FIFO,
	S2LP_TX_SOURCE_GPIO,
	S2LP_TX_SOURCE_PN9
} S2LP_TxSource;

// Generic structure for mantissa and exponent setting.
typedef struct {
	unsigned short mantissa;
	unsigned char exponent;
} S2LP_MantissaExponent;

// FSK deviations (B=4 (high band) and D=1 (REFDIV=0)).
#define S2LP_FDEV_800HZ			((S2LP_MantissaExponent) {129, 0}) // For fXO=26MHz.
#define S2LP_FDEV_2KHZ			((S2LP_MantissaExponent) {323, 0}) // For fXO=26MHz.

// Data rates.
#define S2LP_DATARATE_500BPS	((S2LP_MantissaExponent) {17059, 1}) // Setting for uplink 100bps and fXO=26MHz.
#define S2LP_DATARATE_600BPS	((S2LP_MantissaExponent) {33579, 1}) // For fXO=26MHz.

// RX bandwidths.
#define S2LP_RXBW_1KHZ			((S2LP_MantissaExponent) {8, 9})

// Preamble patterns.
typedef enum {
	S2LP_PREAMBLE_PATTERN_0101 = 0x00,
	S2LP_PREAMBLE_PATTERN_1010,
	S2LP_PREAMBLE_PATTERN_1100,
	S2LP_PREAMBLE_PATTERN_0011
} S2LP_PreamblePattern;

/*** S2LP functions ***/

// Common functions.
void S2LP_Init(void);
void S2LP_DisableGpio(void);
void S2LP_Tcxo(unsigned char tcxo_power_enable);
void S2LP_SendCommand(S2LP_Command command);
void S2LP_SetOscillator(S2LP_Oscillator s2lp_oscillator);
void S2LP_ConfigureSmps(void);
void S2LP_ConfigureChargePump(void);
void S2LP_SetModulation(S2LP_Modulation modulation);
void S2LP_SetRfFrequency(unsigned int rf_frequency_hz);
void S2LP_SetFskDeviation(S2LP_MantissaExponent fsk_deviation_setting);
void S2LP_SetBitRate(S2LP_MantissaExponent bit_rate_setting);
void S2LP_ConfigureGpio(unsigned char gpio_number, S2LP_GPIO_Mode gpio_mode, unsigned char gpio_function, unsigned char fifo_flag_direction);
void S2LP_SetFifoThreshold(S2LP_FifoThreshold fifo_threshold, unsigned char threshold_value);
unsigned int S2LP_GetIrqFlags(void);

// TX functions.
void S2LP_ConfigurePa(void);
void S2LP_SetTxSource(S2LP_TxSource tx_source);
void S2LP_WriteFifo(unsigned char* tx_data, unsigned char tx_data_length_bytes);

// RX functions.
void S2LP_SetRxBandwidth(S2LP_MantissaExponent rxbw_setting);
void S2LP_SetPreambleDetector(unsigned char preamble_length_bytes, S2LP_PreamblePattern preamble_pattern);
void S2LP_SetSyncWord(unsigned char* sync_word, unsigned char sync_word_length_bits);
void S2LP_SetRxDataLength(unsigned char rx_data_length_bytes);
signed char S2LP_GetRssi(void);
void S2LP_ReadFifo(unsigned char* rx_data, unsigned char rx_data_length_bytes);

#endif /* S2LP_H */
