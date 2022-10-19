/*
 * s2lp.h
 *
 *  Created on: 16 aug. 2019
 *      Author: Ludo
 */

#ifndef __S2LP_H__
#define __S2LP_H__

#include "lptim.h"
#include "s2lp_reg.h"
#include "spi.h"
#include "types.h"

/*** S2LP macros ***/

#define S2LP_FIFO_SIZE_BYTES	128

/*** S2LP structures ***/

typedef enum {
	S2LP_SUCCESS = 0,
	S2LP_ERROR_NULL_PARAMETER,
	S2LP_ERROR_COMMAND,
	S2LP_ERROR_STATE_TIMEOUT,
	S2LP_ERROR_OSCILLATOR_TIMEOUT,
	S2LP_ERROR_OSCILLATOR,
	S2LP_ERROR_MODULATION,
	S2LP_ERROR_RF_FREQUENCY_OVERFLOW,
	S2LP_ERROR_RF_FREQUENCY_UNDERFLOW,
	S2LP_ERROR_GPIO_INDEX,
	S2LP_ERROR_GPIO_MODE,
	S2LP_ERROR_GPIO_FUNCTION,
	S2LP_ERROR_FIFO_FLAG_DIRECTION,
	S2LP_ERROR_FIFO_THRESHOLD,
	S2LP_ERROR_FIFO_THRESHOLD_VALUE,
	S2LP_ERROR_IRQ_INDEX,
	S2LP_ERROR_PREAMBLE_PATTERN,
	S2LP_ERROR_SYNC_WORD_LENGTH,
	S2LP_ERROR_RF_OUTPUT_POWER_OVERFLOW,
	S2LP_ERROR_RF_OUTPUT_POWER_UNDERFLOW,
	S2LP_ERROR_TX_SOURCE,
	S2LP_ERROR_TX_DATA_LENGTH,
	S2LP_ERROR_RX_SOURCE,
	S2LP_ERROR_RX_DATA_LENGTH,
	S2LP_ERROR_BASE_SPI = 0x0100,
	S2LP_ERROR_BASE_LPTIM = (S2LP_ERROR_BASE_SPI + SPI_ERROR_BASE_LAST),
	S2LP_ERROR_BASE_LAST = (S2LP_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} S2LP_status_t;

// Chip state.
typedef enum {
	S2LP_STATE_READY = 0x00,
	S2LP_STATE_SLEEP_A,
	S2LP_STATE_STANDBY,
	S2LP_STATE_SLEEP_B,
	S2LP_STATE_LOCK = 0x0C,
	S2LP_STATE_RX = 0x30,
	S2LP_STATE_SYNTH_SETUP = 0x50,
	S2LP_STATE_TX = 0x5C,
	S2LP_STATE_LAST
} S2LP_state_t;

// Oscillator.
typedef enum {
	S2LP_OSCILLATOR_QUARTZ = 0x00,
	S2LP_OSCILLATOR_TCXO,
	S2LP_OSCILLATOR_LAST
} S2LP_oscillator_t;

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
	S2LP_MODULATION_4GFSK_BT05,
	S2LP_MODULATION_LAST
} S2LP_modulation_t;

// GPIOs modes.
typedef enum {
	S2LP_GPIO_MODE_IN = 0x01,
	S2LP_GPIO_MODE_OUT_LOW_POWER,
	S2LP_GPIO_MODE_OUT_HIGH_POWER,
	S2LP_GPIO_MODE_LAST
} S2LP_gpio_mode_t;

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
	S2LP_GPIO_OUTPUT_FUNCTION_LAST
} S2LP_gpio_output_function_t;

// GPIO input functions.
typedef enum {
	S2LP_GPIO_INPUT_FUNCTION_TX_COMMAND,
	S2LP_GPIO_INPUT_FUNCTION_RX_COMMAND,
	S2LP_GPIO_INPUT_FUNCTION_TX_DATA,
	S2LP_GPIO_INPUT_FUNCTION_WAKE_UP,
	S2LP_GPIO_INPUT_FUNCTION_EXT_CLOCK,
	S2LP_GPIO_INPUT_FUNCTION_LAST
} S2LP_gpio_input_function_t;

typedef enum {
	S2LP_FIFO_FLAG_DIRECTION_TX = 0x00,
	S2LP_FIFO_FLAG_DIRECTION_RX,
	S2LP_FIFO_FLAG_DIRECTION_LAST
} S2LP_fifo_flag_direction_t;

typedef enum {
	S2LP_FIFO_THRESHOLD_RX_FULL = 0x3C,
	S2LP_FIFO_THRESHOLD_RX_EMPTY,
	S2LP_FIFO_THRESHOLD_TX_FULL,
	S2LP_FIFO_THRESHOLD_TX_EMPTY,
	S2LP_FIFO_THRESHOLD_LAST
} S2LP_fifo_threshold_t;

typedef enum {
	S2LP_TX_SOURCE_NORMAL = 0x00,
	S2LP_TX_SOURCE_FIFO,
	S2LP_TX_SOURCE_GPIO,
	S2LP_TX_SOURCE_PN9,
	S2LP_TX_SOURCE_LAST
} S2LP_tx_source_t;

typedef enum {
	S2LP_RX_SOURCE_NORMAL = 0x00,
	S2LP_RX_SOURCE_FIFO,
	S2LP_RX_SOURCE_GPIO,
	S2LP_RX_SOURCE_LAST
} S2LP_rx_source_t;

// Interrupts.
typedef enum {
	S2LP_IRQ_INDEX_RX_DATA_READY = 0,
	S2LP_IRQ_INDEX_RX_DATA_DISC,
	S2LP_IRQ_INDEX_TX_DATA_SENT,
	S2LP_IRQ_INDEX_MAX_RE_TX_REACH,
	S2LP_IRQ_INDEX_CRC_ERROR,
	S2LP_IRQ_INDEX_TX_FIFO_ERROR,
	S2LP_IRQ_INDEX_RX_FIFO_ERROR,
	S2LP_IRQ_INDEX_TX_FIFO_ALMOST_FULL,
	S2LP_IRQ_INDEX_TX_FIFO_ALMOST_EMPTY,
	S2LP_IRQ_INDEX_RX_FIFO_ALMOST_FULL,
	S2LP_IRQ_INDEX_RX_FIFO_ALMOST_EMPTY,
	S2LP_IRQ_INDEX_MAX_BO_CCA_REACH,
	S2LP_IRQ_INDEX_VALID_PREAMBLE,
	S2LP_IRQ_INDEX_VALID_SYNC,
	S2LP_IRQ_INDEX_RSSI_ABOVE_TH,
	S2LP_IRQ_INDEX_WKUP_TOUT_LDC,
	S2LP_IRQ_INDEX_READY,
	S2LP_IRQ_INDEX_STANDBY_DELAYED,
	S2LP_IRQ_INDEX_LOW_BATT_LVL,
	S2LP_IRQ_INDEX_POR,
	S2LP_IRQ_INDEX_RX_TIMEOUT = 28,
	S2LP_IRQ_INDEX_RX_SNIFF_TIMEOUT,
	S2LP_IRQ_INDEX_LAST
} S2LP_irq_index_t;

// Generic structure for mantissa and exponent setting.
typedef struct {
	uint16_t mantissa;
	uint8_t exponent;
} S2LP_mantissa_exponent_t;

// FSK deviations (B=4 (high band) and D=1 (REFDIV=0)).
#define S2LP_FDEV_2KHZ			((S2LP_mantissa_exponent_t) {171, 0}) // Setting for uplink 100bps and fXO=49.152MHz.
#define S2LP_FDEV_800HZ			((S2LP_mantissa_exponent_t) {68, 0}) // Setting for downlink 600bps and fXO=49.152MHz.

// Data rates.
#define S2LP_DATARATE_500BPS	((S2LP_mantissa_exponent_t) {21845, 1}) // Setting for uplink 100bps and fXO=49.152MHz.
#define S2LP_DATARATE_600BPS	((S2LP_mantissa_exponent_t) {39322, 1}) // Setting for downlink 600bps and fXO=49.152MHz.

// RX bandwidths.
#define S2LP_RXBW_2KHZ1			((S2LP_mantissa_exponent_t) {8, 8})

// Preamble patterns.
typedef enum {
	S2LP_PREAMBLE_PATTERN_0101 = 0x00,
	S2LP_PREAMBLE_PATTERN_1010,
	S2LP_PREAMBLE_PATTERN_1100,
	S2LP_PREAMBLE_PATTERN_0011,
	S2LP_PREAMBLE_PATTERN_LAST
} S2LP_preamble_pattern_t;

// SMPS setting.
typedef struct {
	uint8_t s2lp_smps_reg_pm_conf3;
	uint8_t s2lp_smps_reg_pm_conf2;
} S2LP_smps_setting_t;

// SMPS frequencies.
#define S2LP_SMPS_TX			((S2LP_smps_setting_t) {0x9C, 0x28})
#define S2LP_SMPS_RX			((S2LP_smps_setting_t) {0x87, 0xFC})

/*** S2LP functions ***/

// GPIOs functions.
void S2LP_init(void);
void S2LP_disable(void);
S2LP_status_t S2LP_tcxo(uint8_t tcxo_enable);
S2LP_status_t S2LP_shutdown(uint8_t shutdown_enable);

// Common functions.
S2LP_status_t S2LP_send_command(S2LP_command_t command);
S2LP_status_t S2LP_wait_for_state(S2LP_state_t new_state);
S2LP_status_t S2LP_wait_for_oscillator(void);
S2LP_status_t S2LP_set_oscillator(S2LP_oscillator_t s2lp_oscillator);
S2LP_status_t S2LP_configure_smps(S2LP_smps_setting_t smps_setting);
S2LP_status_t S2LP_configure_charge_pump(void);
S2LP_status_t S2LP_set_modulation(S2LP_modulation_t modulation);
S2LP_status_t S2LP_set_rf_frequency(uint32_t rf_frequency_hz);
S2LP_status_t S2LP_set_fsk_deviation(S2LP_mantissa_exponent_t fsk_deviation_setting);
S2LP_status_t S2LP_set_bitrate(S2LP_mantissa_exponent_t bit_rate_setting);
S2LP_status_t S2LP_configure_gpio(uint8_t gpio_index, S2LP_gpio_mode_t gpio_mode, uint8_t gpio_function, S2LP_fifo_flag_direction_t fifo_flag_direction);
S2LP_status_t S2LP_set_fifo_threshold(S2LP_fifo_threshold_t fifo_threshold, uint8_t threshold_value);
S2LP_status_t S2LP_configure_irq(S2LP_irq_index_t irq_index, uint8_t irq_enable);
S2LP_status_t S2LP_clear_irq_flags(void);

// Packet functions.
S2LP_status_t S2LP_set_packet_length(uint8_t packet_length_bytes);
S2LP_status_t S2LP_set_preamble_detector(uint8_t preamble_length_2bits, S2LP_preamble_pattern_t preamble_pattern);
S2LP_status_t S2LP_set_sync_word(uint8_t* sync_word, uint8_t sync_word_length_bits);
S2LP_status_t S2LP_disable_crc(void);

// TX functions.
S2LP_status_t S2LP_configure_pa(void);
S2LP_status_t S2LP_set_rf_output_power(int8_t output_power_dbm);
S2LP_status_t S2LP_set_tx_source(S2LP_tx_source_t tx_source);
S2LP_status_t S2LP_write_fifo(uint8_t* tx_data, uint8_t tx_data_length_bytes);

// RX functions.
S2LP_status_t S2LP_set_rx_source(S2LP_rx_source_t rx_source);
S2LP_status_t S2LP_set_rx_bandwidth(S2LP_mantissa_exponent_t rxbw_setting);
S2LP_status_t S2LP_disable_equa_cs_ant_switch(void);
S2LP_status_t S2LP_get_rssi(int16_t* rssi_dbm);
S2LP_status_t S2LP_read_fifo(uint8_t* rx_data, uint8_t rx_data_length_bytes);

#define S2LP_status_check(error_base) { if (s2lp_status != S2LP_SUCCESS) { status = error_base + s2lp_status; goto errors; }}
#define S2LP_error_check() { ERROR_status_check(s2lp_status, S2LP_SUCCESS, ERROR_BASE_S2LP); }
#define S2LP_error_check_print() { ERROR_status_check_print(s2lp_status, S2LP_SUCCESS, ERROR_BASE_S2LP); }

#endif /* __S2LP_H__ */
