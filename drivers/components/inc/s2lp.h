/*
 * s2lp.h
 *
 *  Created on: 16 aug. 2019
 *      Author: Ludo
 */

#ifndef __S2LP_H__
#define __S2LP_H__

#include "exti.h"
#include "lptim.h"
#include "s2lp_reg.h"
#include "spi.h"
#include "types.h"

/*** S2LP macros ***/

#define S2LP_FIFO_SIZE_BYTES	128
#define S2LP_SHUTDOWN_DELAY_MS	50

/*** S2LP structures ***/

/*!******************************************************************
 * \enum S2LP_status_t
 * \brief S2LP driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	S2LP_SUCCESS = 0,
	S2LP_ERROR_NULL_PARAMETER,
	S2LP_ERROR_COMMAND,
	S2LP_ERROR_STATE_TIMEOUT,
	S2LP_ERROR_RADIO_PATH,
	S2LP_ERROR_OSCILLATOR_TIMEOUT,
	S2LP_ERROR_OSCILLATOR,
	S2LP_ERROR_MODULATION,
	S2LP_ERROR_SMPS_FREQUENCY_OVERFLOW,
	S2LP_ERROR_RF_FREQUENCY_OVERFLOW,
	S2LP_ERROR_RF_FREQUENCY_UNDERFLOW,
	S2LP_ERROR_DEVIATION_UNDERFLOW,
	S2LP_ERROR_DEVIATION_OVERFLOW,
	S2LP_ERROR_RX_BANDWIDTH_UNDERFLOW,
	S2LP_ERROR_RX_BANDWIDTH_OVERFLOW,
	S2LP_ERROR_DATARATE_OVERFLOW,
	S2LP_ERROR_DATARATE_UNDERFLOW,
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
	S2LP_ERROR_RSSI_THRESHOLD,
	S2LP_ERROR_RSSI_TYPE,
	// Low level drivers errors.
	S2LP_ERROR_BASE_SPI1 = 0x0100,
	S2LP_ERROR_BASE_LPTIM1 = (S2LP_ERROR_BASE_SPI1 + SPI_ERROR_BASE_LAST),
	// Last base value.
	S2LP_ERROR_BASE_LAST = (S2LP_ERROR_BASE_LPTIM1 + LPTIM_ERROR_BASE_LAST)
} S2LP_status_t;

/*!******************************************************************
 * \enum S2LP_command_t
 * \brief S2LP commands list.
 *******************************************************************/
typedef enum {
	S2LP_COMMAND_TX = 0x60,
	S2LP_COMMAND_RX,
	S2LP_COMMAND_READY,
	S2LP_COMMAND_STANDBY,
	S2LP_COMMAND_SLEEP,
	S2LP_COMMAND_LOCKRX,
	S2LP_COMMAND_LOCKTX,
	S2LP_COMMAND_SABORT,
	S2LP_COMMAND_LDC_RELOAD,
	S2LP_COMMAND_SRES = 0x70,
	S2LP_COMMAND_FLUSHRXFIFO,
	S2LP_COMMAND_FLUSHTXFIFO,
	S2LP_COMMAND_SEQUENCE_UPDATE,
	S2LP_COMMAND_LAST
} S2LP_command_t;

/*!******************************************************************
 * \enum S2LP_state_t
 * \brief S2LP internal states.
 *******************************************************************/
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

/*!******************************************************************
 * \enum S2LP_oscillator_t
 * \brief S2LP external oscillator type.
 *******************************************************************/
typedef enum {
	S2LP_OSCILLATOR_QUARTZ = 0x00,
	S2LP_OSCILLATOR_TCXO,
	S2LP_OSCILLATOR_LAST
} S2LP_oscillator_t;

/*!******************************************************************
 * \enum S2LP_modulation_t
 * \brief S2LP modulation schemes.
 *******************************************************************/
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

/*!******************************************************************
 * \enum S2LP_gpio_t
 * \brief S2LP GPIO pins list.
 *******************************************************************/
typedef enum {
	S2LP_GPIO0 = 0,
	S2LP_GPIO1,
	S2LP_GPIO2,
	S2LP_GPIO3,
	S2LP_GPIO_LAST
} S2LP_gpio_t;

/*!******************************************************************
 * \enum S2LP_gpio_mode_t
 * \brief S2LP GPIO modes.
 *******************************************************************/
typedef enum {
	S2LP_GPIO_MODE_IN = 0x01,
	S2LP_GPIO_MODE_OUT_LOW_POWER,
	S2LP_GPIO_MODE_OUT_HIGH_POWER,
	S2LP_GPIO_MODE_LAST
} S2LP_gpio_mode_t;

/*!******************************************************************
 * \enum S2LP_gpio_output_function_t
 * \brief S2LP GPIO output functions.
 *******************************************************************/
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

/*!******************************************************************
 * \enum S2LP_gpio_input_function_t
 * \brief S2LP GPIO input functions.
 *******************************************************************/
typedef enum {
	S2LP_GPIO_INPUT_FUNCTION_TX_COMMAND,
	S2LP_GPIO_INPUT_FUNCTION_RX_COMMAND,
	S2LP_GPIO_INPUT_FUNCTION_TX_DATA,
	S2LP_GPIO_INPUT_FUNCTION_WAKE_UP,
	S2LP_GPIO_INPUT_FUNCTION_EXT_CLOCK,
	S2LP_GPIO_INPUT_FUNCTION_LAST
} S2LP_gpio_input_function_t;

/*!******************************************************************
 * \enum S2LP_fifo_flag_direction_t
 * \brief S2LP FIFO flag direction.
 *******************************************************************/
typedef enum {
	S2LP_FIFO_FLAG_DIRECTION_TX = 0,
	S2LP_FIFO_FLAG_DIRECTION_RX,
	S2LP_FIFO_FLAG_DIRECTION_LAST
} S2LP_fifo_flag_direction_t;

/*!******************************************************************
 * \enum S2LP_fifo_threshold_t
 * \brief S2LP FIFO thresholds list.
 *******************************************************************/
typedef enum {
	S2LP_FIFO_THRESHOLD_RX_FULL = 0x3C,
	S2LP_FIFO_THRESHOLD_RX_EMPTY,
	S2LP_FIFO_THRESHOLD_TX_FULL,
	S2LP_FIFO_THRESHOLD_TX_EMPTY,
	S2LP_FIFO_THRESHOLD_LAST
} S2LP_fifo_threshold_t;

/*!******************************************************************
 * \enum S2LP_tx_source_t
 * \brief S2LP TX data sources.
 *******************************************************************/
typedef enum {
	S2LP_TX_SOURCE_NORMAL = 0x00,
	S2LP_TX_SOURCE_FIFO,
	S2LP_TX_SOURCE_GPIO,
	S2LP_TX_SOURCE_PN9,
	S2LP_TX_SOURCE_LAST
} S2LP_tx_source_t;

/*!******************************************************************
 * \enum S2LP_rx_source_t
 * \brief S2LP RX data sources.
 *******************************************************************/
typedef enum {
	S2LP_RX_SOURCE_NORMAL = 0x00,
	S2LP_RX_SOURCE_FIFO,
	S2LP_RX_SOURCE_GPIO,
	S2LP_RX_SOURCE_LAST
} S2LP_rx_source_t;

/*!******************************************************************
 * \enum S2LP_irq_index_t
 * \brief S2LP internal interrupts list.
 *******************************************************************/
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

/*!******************************************************************
 * \enum S2LP_rssi_t
 * \brief S2LP RSSI measurement methods.
 *******************************************************************/
typedef enum {
	S2LP_RSSI_TYPE_RUN,
	S2LP_RSSI_TYPE_SYNC_WORD,
	S2LP_RSSI_TYPE_LAST
} S2LP_rssi_t;

/*!******************************************************************
 * \enum S2LP_preamble_pattern_t
 * \brief S2LP pre-defined preamble patterns.
 *******************************************************************/
typedef enum {
	S2LP_PREAMBLE_PATTERN_0101 = 0x00,
	S2LP_PREAMBLE_PATTERN_1010,
	S2LP_PREAMBLE_PATTERN_1100,
	S2LP_PREAMBLE_PATTERN_0011,
	S2LP_PREAMBLE_PATTERN_LAST
} S2LP_preamble_pattern_t;

/*** S2LP functions ***/

/*!******************************************************************
 * \fn void S2LP_init(void)
 * \brief Init S2LP interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void S2LP_init(void);

/*!******************************************************************
 * \fn void S2LP_de_init(void)
 * \brief Release S2LP interface.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void S2LP_de_init(void);

#ifdef HW1_1
/*!******************************************************************
 * \fn S2LP_status_t S2LP_shutdown(uint8_t shutdown_enable)
 * \brief Control S2LP shutdown pin.
 * \param[in]  	shutdown_enable: 0 to exit shutdown mode, any other value to enter.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_shutdown(uint8_t shutdown_enable);
#endif

/*!******************************************************************
 * \fn S2LP_status_t S2LP_send_command(S2LP_command_t command)
 * \brief Send a command to S2LP.
 * \param[in]  	command: Command to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_send_command(S2LP_command_t command);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_wait_for_state(S2LP_state_t new_state)
 * \brief Wait for S2LP to be in the given state.
 * \param[in]  	new_state: Expected state to wait for.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_wait_for_state(S2LP_state_t new_state);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_oscillator(S2LP_oscillator_t oscillator)
 * \brief Set S2LP oscillator type.
 * \param[in]  	oscillator: Oscillator type.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_oscillator(S2LP_oscillator_t oscillator);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_wait_for_oscillator(void)
 * \brief Wait for S2LP oscillator to be ready.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_wait_for_oscillator(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_configure_charge_pump(void
 * \brief Configure S2LP internal charge pump.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_configure_charge_pump(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_smps_frequency(uint32_t frequency_hz)
 * \brief Set S2LP internal DC-DC- switching frequency.
 * \param[in]  	frequency_hz: DC-DC switching frequency in Hz.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_smps_frequency(uint32_t frequency_hz);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_modulation(S2LP_modulation_t modulation)
 * \brief Set S2LP modulation scheme.
 * \param[in]  	modulation: Modulation to select.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_modulation(S2LP_modulation_t modulation);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_rf_frequency(uint32_t frequency_hz)
 * \brief Set S2LP RF center frequency.
 * \param[in]  	frequency_hz: RF center frequency in Hz.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_rf_frequency(uint32_t frequency_hz);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_fsk_deviation(uint32_t deviation_hz)
 * \brief Set S2LP frequency deviation for frequency modulations.
 * \param[in]  	deviation_hz: Frequency deviation in Hz.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_fsk_deviation(uint32_t deviation_hz);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_datarate(uint32_t datarate_bps)
 * \brief Set S2LP data rate.
 * \param[in]  	datarate_bps: Datarate in bps.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_datarate(uint32_t datarate_bps);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_configure_pa(void)
 * \brief Configure S2LP power amplifier.
 * \param[in]	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_configure_pa(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_rf_output_power(int8_t output_power_dbm)
 * \brief Set S2LP transmission output power.
 * \param[in]	output_power_dbm: Output power in dBm.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_rf_output_power(int8_t output_power_dbm);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_tx_source(S2LP_tx_source_t tx_source)
 * \brief Set S2LP transmission data source.
 * \param[in]	tx_source: TX data source.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_tx_source(S2LP_tx_source_t tx_source);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_rx_source(S2LP_rx_source_t rx_source)
 * \brief Set S2LP reception data source.
 * \param[in]	rx_source: RX data source.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_rx_source(S2LP_rx_source_t rx_source);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_rx_bandwidth(uint32_t rx_bandwidth_hz)
 * \brief Set S2LP receiver bandwidth.
 * \param[in]	rx_bandwidth_hz: RX channel bandwidth in Hz
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_rx_bandwidth(uint32_t rx_bandwidth_hz);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_disable_equa_cs_ant_switch(void)
 * \brief Disable S2LP equalizer, CS blanking and antenna switch.
 * \param[in]	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_disable_equa_cs_ant_switch(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_disable_afc(void)
 * \brief Disable S2LP AFC.
 * \param[in]	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_disable_afc(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_configure_clock_recovery(void)
 * \brief Configure S2LP clock recovery system.
 * \param[in]	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_configure_clock_recovery(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_rssi_threshold(int16_t rssi_threshold_dbm)
 * \brief Set S2LP RSSI detection threshold.
 * \param[in]	rssi_threshold_dbm: RSSI threshold in dBm.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_rssi_threshold(int16_t rssi_threshold_dbm);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_get_rssi(S2LP_rssi_t rssi_type, int16_t* rssi_dbm)
 * \brief Read S2LP RSSI.
 * \param[in]	rssi_type: RSSI type to read.
 * \param[out] 	rssi_dbm: Pointer to signed short that will contain RSSI in dBm.
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_get_rssi(S2LP_rssi_t rssi_type, int16_t* rssi_dbm);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_configure_gpio(S2LP_gpio_t gpio, S2LP_gpio_mode_t mode, uint8_t function, S2LP_fifo_flag_direction_t fifo_flag_direction)
 * \brief Configure S2LP GPIO function.
 * \param[in]  	gpio: GPIO to configure.
 * \param[in]	mode: GPIO mode to set.
 * \param[in]	function: GPIO function to set.
 * \param[in]	fifo_flag_direction: FIFO flag direction.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_configure_gpio(S2LP_gpio_t gpio, S2LP_gpio_mode_t mode, uint8_t function, S2LP_fifo_flag_direction_t fifo_flag_direction);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_enable_nirq(S2LP_fifo_flag_direction_t fifo_flag_direction, EXTI_gpio_irq_cb_t irq_callback)
 * \brief Configure and enable the NIRQ GPIO link between S2LP and MCU.
 * \param[in]  	fifo_flag_direction: FIFO flag direction.
 * \param[in]	irq_callback: Function to call on NIRQ event.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_enable_nirq(S2LP_fifo_flag_direction_t fifo_flag_direction, EXTI_gpio_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn void S2LP_disable_nirq(void)
 * \brief Disable the NIRQ GPIO link between S2LP and MCU.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void S2LP_disable_nirq(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_configure_irq(S2LP_irq_index_t irq_index, uint8_t irq_enable)
 * \brief Configure S2LP internal interrupt.
 * \param[in]  	irq_index: Interrupt index.
 * \param[in]	irq_enable: 0 to disable, any other value to enable.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_configure_irq(S2LP_irq_index_t irq_index, uint8_t irq_enable);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_get_irq_flag(S2LP_irq_index_t irq_index, uint8_t* irq_flag)
 * \brief Read S2LP internal interrupt status.
 * \param[in]  	irq_index: Interrupt index.
 * \param[out] 	irq_flag: Pointer to bit that will contain interrupt status.
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_get_irq_flag(S2LP_irq_index_t irq_index, uint8_t* irq_flag);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_disable_all_irq(void)
 * \brief Disable all S2LP internal interrupts.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_disable_all_irq(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_clear_all_irq(void)
 * \brief Clear all S2LP internal interrupts flags.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_clear_all_irq(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_packet_length(uint8_t packet_length_bytes)
 * \brief Set S2LP decoder packet length.
 * \param[in]	packet_length_bytes: Packet length in bytes.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_packet_length(uint8_t packet_length_bytes);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_preamble_detector(uint8_t preamble_length_2bits, S2LP_preamble_pattern_t preamble_pattern)
 * \brief Configure S2LP preamble detector.
 * \param[in]  	preamble_length_2bits: Number of bits pair of the preamble.
 * \param[in]	preamble_pattern: Preamble pattern to search for.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_preamble_detector(uint8_t preamble_length_2bits, S2LP_preamble_pattern_t preamble_pattern);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_sync_word(uint8_t* sync_word, uint8_t sync_word_length_bits)
 * \brief Configure S2LP synchronization word detector.
 * \param[in]  	sync_word: Synchronization word.
 * \param[in]	sync_word_length_bits: Length of the synchronization word in bits.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_sync_word(uint8_t* sync_word, uint8_t sync_word_length_bits);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_disable_crc(void)
 * \brief Disable S2LP CRC computation.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_disable_crc(void);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_set_fifo_threshold(S2LP_fifo_threshold_t fifo_threshold, uint8_t threshold_value)
 * \brief Set S2LP internal FIFO threshold.
 * \param[in]  	fifo_threshold: Threshold to set.
 * \param[in]	threshold_value: Threshold value in bytes.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_set_fifo_threshold(S2LP_fifo_threshold_t fifo_threshold, uint8_t threshold_value);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_write_fifo(uint8_t* tx_data, uint8_t tx_data_length_bytes)
 * \brief Write S2LP internal FIFO.
 * \param[in]  	tx_data: Data to write.
 * \param[in]	tx_data_length_bytes: Number of bytes to write (size of tx_data).
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_write_fifo(uint8_t* tx_data, uint8_t tx_data_length_bytes);

/*!******************************************************************
 * \fn S2LP_status_t S2LP_read_fifo(uint8_t* rx_data, uint8_t rx_data_length_bytes)
 * \brief Read S2LP internal FIFO.
 * \param[in]  	rx_data: Pointer to byte array that will contain FIFO data.
 * \param[in]	rx_data_length_bytes: Number of bytes to read (size of rx_data).
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
S2LP_status_t S2LP_read_fifo(uint8_t* rx_data, uint8_t rx_data_length_bytes);

/*******************************************************************/
#define S2LP_exit_error(base) { ERROR_check_exit(s2lp_status, S2LP_SUCCESS, base) }

/*******************************************************************/
#define S2LP_stack_error(base) { ERROR_check_stack(s2lp_status, S2LP_SUCCESS, base) }

/*******************************************************************/
#define S2LP_stack_exit_error(base, code) { ERROR_check_stack_exit(s2lp_status, S2LP_SUCCESS, base, code) }

#endif /* __S2LP_H__ */
