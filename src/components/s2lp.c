/*
 * s2lp.c
 *
 *  Created on: 16 aug. 2019
 *      Author: Ludo
 */

#include "s2lp.h"

#include "dma.h"
#include "exti.h"
#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "pwr.h"
#include "s2lp_reg.h"
#include "spi.h"

/*** S2LP local macros ***/

// SPI header bytes.
#define S2LP_HEADER_BYTE_WRITE				0x00
#define S2LP_HEADER_BYTE_READ				0x01
#define S2LP_HEADER_BYTE_COMMAND			0x80
// State waiting timeout.
#define S2LP_TIMEOUT_MS						1000
#define S2LP_TIMEOUT_SUB_DELAY_MS			10
// Crystal frequency ranges.
#define S2LP_XO_FREQUENCY_HZ				49152000
#define S2LP_XO_HIGH_RANGE_THRESHOLD_HZ		48000000
// RF frequency range.
#define S2LP_RF_FREQUENCY_HZ_MIN			826000000
#define S2LP_RF_FREQUENCY_HZ_MAX			958000000
// RF output power range.
#define S2LP_RF_OUTPUT_POWER_MIN			-49
#define S2LP_RF_OUTPUT_POWER_MAX			14
// Sync word max length.
#define S2LP_SYNC_WORD_LENGTH_BITS_MAX		32
// RSSI offset.
#define S2LP_RSSI_OFFSET_DB					146
// FIFO.
#define S2LP_TX_FIFO_USE_DMA // Use DMA to fill TX FIFO if defined, standard SPI access otherwise.
#define S2LP_FIFO_THHRESHOLD_BYTES_MAX		0x7F
// Last register address
#define S2LP_REGISTER_ADRESS_LAST			0x7F
// Number of GPIOs.
#define S2LP_NUMBER_OF_GPIO					4





/*** S2LP local functions ***/

/* S2LP REGISTER WRITE FUNCTION.
 * @param addr:		Register address.
 * @param value:	Value to write in register.
 * @return status:	Function execution status.
 */
static S2LP_status_t S2LP_write_register(unsigned char addr, unsigned char value) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	// Falling edge on CS pin.
	GPIO_write(&GPIO_S2LP_CS, 0);
	// Write sequence.
	spi_status = SPI1_write_byte(S2LP_HEADER_BYTE_WRITE); // A/C='0' and W/R='0'.
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
	spi_status = SPI1_write_byte(addr);
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
	spi_status = SPI1_write_byte(value);
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
errors:
	GPIO_write(&GPIO_S2LP_CS, 1); // Set CS pin.
	return status;
}

/* S2LP REGISTER READ FUNCTION.
 * @param addr:		Register address.
 * @param value:	Pointer to byte that will contain the register value to read.
 * @return status:	Function execution status.
 */
static S2LP_status_t S2LP_read_register(unsigned char addr, unsigned char* value) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	// Falling edge on CS pin.
	GPIO_write(&GPIO_S2LP_CS, 0);
	// Read sequence.
	spi_status = SPI1_write_byte(S2LP_HEADER_BYTE_READ); // A/C='0' and W/R='1'.
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
	spi_status = SPI1_write_byte(addr);
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
	spi_status = SPI1_read_byte(0xFF, value);
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
errors:
	GPIO_write(&GPIO_S2LP_CS, 1); // Set CS pin.
	return status;
}

/*** S2LP functions ***/

/* INIT S2LP INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void S2LP_init(void) {
	// Configure TCXO power control pin.
	GPIO_configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* CONTROL EXTERNAL TCXO.
 * @param:			None.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_tcxo(unsigned char tcxo_enable) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Turn TCXO on or off.
	GPIO_write(&GPIO_TCXO_POWER_ENABLE, tcxo_enable);
	lptim1_status = LPTIM1_delay_milliseconds(100, 1);
	LPTIM1_status_check(S2LP_ERROR_BASE_LPTIM);
errors:
	return status;
}

/* PUT S2LP IN SHUTDOWN OR ACTIVE MODE.
 * @param shutdown_enable:	0 to exit shutdown mode, any other value to enter shutdown mode.
 * @return status:			Function execution status.
 */
S2LP_status_t S2LP_shutdown(unsigned char shutdown_enable) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
#ifdef HW1_1
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Configure GPIO.
	if (shutdown_enable == 0) {
		// Put SDN low.
		GPIO_configure(&GPIO_S2LP_SDN, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
		// Wait for reset time.
		lptim1_status = LPTIM1_delay_milliseconds(100, 1);
		LPTIM1_status_check(S2LP_ERROR_BASE_LPTIM);
	}
	else {
		// Put SDN in high impedance (pull-up resistor used).
		GPIO_configure(&GPIO_S2LP_SDN, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	}
errors:
#endif
	return status;
}

/* SEND COMMAND TO S2LP.
 * @param command:	Command to send.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_send_command(S2LP_command_t command) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	// Check command.
	if (command >= S2LP_COMMAND_LAST) {
		status = S2LP_ERROR_COMMAND;
		goto errors;
	}
	// Falling edge on CS pin.
	GPIO_write(&GPIO_S2LP_CS, 0);
	// Write sequence.
	spi_status = SPI1_write_byte(S2LP_HEADER_BYTE_COMMAND); // A/C='1' and W/R='0'.
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
	spi_status = SPI1_write_byte(command);
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
errors:
	GPIO_write(&GPIO_S2LP_CS, 1); // Set CS pin.
	return status;
}

/* WAIT FOR S2LP TO ENTER A GIVEN STATE.
 * @param new_state:	State to reach.
 * @return status:		Function execution status.
 */
S2LP_status_t S2LP_wait_for_state(S2LP_state_t new_state) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	unsigned char state = 0;
	unsigned char reg_value = 0;
	unsigned int delay_ms = 0;
	// Poll MC_STATE until state is reached.
	do {
		status = S2LP_read_register(S2LP_REG_MC_STATE0, &reg_value);
		if (status != S2LP_SUCCESS) goto errors;
		state = (reg_value >> 1) & 0x7F;
		// Internal delay.
		lptim1_status = LPTIM1_delay_milliseconds(S2LP_TIMEOUT_SUB_DELAY_MS, 1);
		LPTIM1_status_check(S2LP_ERROR_BASE_LPTIM);
		// Exit if timeout.
		delay_ms += S2LP_TIMEOUT_SUB_DELAY_MS;
		if (delay_ms > S2LP_TIMEOUT_MS) {
			status = S2LP_ERROR_STATE_TIMEOUT;
			goto errors;
		}
	}
	while (state != new_state);
errors:
	return status;
}

/* WAIT FOR S2LP OSCILLATOR TO BE RUNNING.
 * @param:			None.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_wait_for_oscillator(void) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	unsigned char xo_on = 0;
	unsigned char reg_value = 0;
	unsigned int delay_ms = 0;
	// Poll MC_STATE until XO bit is set.
	do {
		status = S2LP_read_register(S2LP_REG_MC_STATE0, &reg_value);
		if (status != S2LP_SUCCESS) goto errors;
		xo_on = (reg_value & 0x01);
		// Internal delay.
		lptim1_status = LPTIM1_delay_milliseconds(S2LP_TIMEOUT_SUB_DELAY_MS, 1);
		LPTIM1_status_check(S2LP_ERROR_BASE_LPTIM);
		// Exit if timeout.
		delay_ms += S2LP_TIMEOUT_SUB_DELAY_MS;
		if (delay_ms > S2LP_TIMEOUT_MS) {
			status = S2LP_ERROR_OSCILLATOR_TIMEOUT;
			goto errors;
		}
	}
	while (xo_on == 0);
errors:
	return status;
}

/* CONFIGURE S2LP OSCILLATOR.
 * @param s2lp_oscillator:	S2LP oscillator type.
 * @return status:			Function execution status.
 */
S2LP_status_t S2LP_set_oscillator(S2LP_oscillator_t s2lp_oscillator) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	// Check parameter.
	if (s2lp_oscillator >= S2LP_OSCILLATOR_LAST) {
		status = S2LP_ERROR_OSCILLATOR;
		goto errors;
	}
	// Set RFDIV to 0, disable external RCO, configure EXT_REF bit.
	reg_value = (s2lp_oscillator == S2LP_OSCILLATOR_TCXO) ? 0xB0 : 0x30;
	status = S2LP_write_register(S2LP_REG_XO_RCO_CONF0, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Set digital clock divider according to crytal frequency.
	reg_value = (S2LP_XO_FREQUENCY_HZ < S2LP_XO_HIGH_RANGE_THRESHOLD_HZ) ? 0x3E : 0x2E;
	status = S2LP_write_register(S2LP_REG_XO_RCO_CONF1, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* ENABLE INTERNAL DC-DC REGULATOR (SMPS).
 * @param:			None.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_configure_smps(S2LP_smps_setting_t smps_setting) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	// Configure divider and switching frequency.
	status = S2LP_write_register(S2LP_REG_PM_CONF3, smps_setting.s2lp_smps_reg_pm_conf3);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_write_register(S2LP_REG_PM_CONF2, smps_setting.s2lp_smps_reg_pm_conf2);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* CONFIGURE PLL CHARGE-PUMP.
 * @param:			None.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_configure_charge_pump(void) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	// Set PLL_CP_ISEL to '010'.
	status = S2LP_read_register(S2LP_REG_SYNT3, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Set bits.
	reg_value &= 0x1F;
	reg_value |= (0b010 << 5);
	// Write register.
	status = S2LP_write_register(S2LP_REG_SYNT3, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Set PLL_PFD_SPLIT_EN bit according to crystal frequency.
	status = S2LP_read_register(S2LP_REG_SYNTH_CONFIG2, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	reg_value = (S2LP_XO_FREQUENCY_HZ >= S2LP_XO_HIGH_RANGE_THRESHOLD_HZ) ? 0xFB : 0x04;
	status = S2LP_write_register(S2LP_REG_SYNTH_CONFIG2, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET S2LP MODULATION SCHEME.
 * @param modulation:	Selected modulation.
 * @return status:		Function execution status.
 */
S2LP_status_t S2LP_set_modulation(S2LP_modulation_t modulation) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char mod2_reg_value = 0;
	// Check parameter.
	if (modulation >= S2LP_MODULATION_LAST) {
		status = S2LP_ERROR_MODULATION;
		goto errors;
	}
	// Read register.
	status = S2LP_read_register(S2LP_REG_MOD2, &mod2_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Change required bits.
	mod2_reg_value &= 0x0F;
	mod2_reg_value |= (modulation << 4);
	// Write register.
	status = S2LP_write_register(S2LP_REG_MOD2, mod2_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET TRANSCEIVER RF CENTRAL FREQUENCY.
 * @param rf_frequency_hz:	RF frequency in Hz.
 * @return status:			Function execution status.
 */
S2LP_status_t S2LP_set_rf_frequency(unsigned int rf_frequency_hz) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned long long synt_value = 0;
	unsigned char synt_reg_value = 0;
	// Check frequency range.
	if (rf_frequency_hz < S2LP_RF_FREQUENCY_HZ_MIN) {
		status = S2LP_ERROR_RF_FREQUENCY_UNDERFLOW;
		goto errors;
	}
	if (rf_frequency_hz > S2LP_RF_FREQUENCY_HZ_MAX) {
		status = S2LP_ERROR_RF_FREQUENCY_OVERFLOW;
		goto errors;
	}
	// Set IF to 300kHz.
	if (S2LP_XO_FREQUENCY_HZ < S2LP_XO_HIGH_RANGE_THRESHOLD_HZ) {
		status = S2LP_write_register(S2LP_REG_IF_OFFSET_ANA, 0xB8);
		if (status != S2LP_SUCCESS) goto errors;
	}
	// See equation p.27 of S2LP datasheet.
	// Set CHNUM to 0.
	status = S2LP_write_register(S2LP_REG_CHNUM, 0x00);
	if (status != S2LP_SUCCESS) goto errors;
	// B=4 for 868MHz (high band, BS=0). REFDIV was set to 0 in oscillator configuration function.
	// SYNT = (fRF * 2^20 * B/2 * D) / (fXO) = (fRF * 2^21) / (fXO).
	synt_value = 0b1 << 21;
	synt_value *= rf_frequency_hz;
	synt_value /= S2LP_XO_FREQUENCY_HZ;
	// Write registers.
	status = S2LP_read_register(S2LP_REG_SYNT3, &synt_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	synt_reg_value &= 0xE0; // BS=0 to select high band.
	synt_reg_value |= ((synt_value >> 24) & 0x0F);
	status = S2LP_write_register(S2LP_REG_SYNT3, synt_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	synt_reg_value = (synt_value >> 16) & 0xFF;
	status = S2LP_write_register(S2LP_REG_SYNT2, synt_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	synt_reg_value = (synt_value >> 8) & 0xFF;
	status = S2LP_write_register(S2LP_REG_SYNT1, synt_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	synt_reg_value = (synt_value >> 0) & 0xFF;
	status = S2LP_write_register(S2LP_REG_SYNT0, synt_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET FSK DEVIATION.
 * @param fsk_deviation_setting:	FSK deviation mantissa and exponent setting.
 * @return status:					Function execution status.
 */
S2LP_status_t S2LP_set_fsk_deviation(S2LP_mantissa_exponent_t fsk_deviation_setting) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char mod1_reg_value = 0;
	// Write registers.
	status = S2LP_write_register(S2LP_REG_MOD0, fsk_deviation_setting.mantissa);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_read_register(S2LP_REG_MOD1, &mod1_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	mod1_reg_value &= 0xF0;
	mod1_reg_value |= fsk_deviation_setting.exponent;
	status = S2LP_write_register(S2LP_REG_MOD1, mod1_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET DATA BIT RATE.
 * @param bit_rate_setting:	Bit rate FSK deviation mantissa and exponent setting.
 * @return status:			Function execution status.
 */
S2LP_status_t S2LP_set_bitrate(S2LP_mantissa_exponent_t bit_rate_setting) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char mod2_reg_value = 0;
	// Write registers.
	status = S2LP_write_register(S2LP_REG_MOD4, (bit_rate_setting.mantissa >> 8) & 0x00FF);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_write_register(S2LP_REG_MOD3, (bit_rate_setting.mantissa >> 0) & 0x00FF);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_read_register(S2LP_REG_MOD2, &mod2_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	mod2_reg_value &= 0xF0;
	mod2_reg_value |= (bit_rate_setting.exponent);
	status = S2LP_write_register(S2LP_REG_MOD2, mod2_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* CONFIGURE S2LP GPIOs.
 * @param gpio_index:			GPIO to configure (0 to 3).
 * @param gpio_mode:			GPIO mode.
 * @param gpio_function:		GPIO function.
 * @param fifo_flag_direction:	Selects TX or RX FIFO flags.
 * @return status:				Function execution status.
 */
S2LP_status_t S2LP_configure_gpio(unsigned char gpio_index, S2LP_gpio_mode_t gpio_mode, unsigned char gpio_function, S2LP_fifo_flag_direction_t fifo_flag_direction) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	// Check parameters.
	if (gpio_index >= S2LP_NUMBER_OF_GPIO) {
		status = S2LP_ERROR_GPIO_INDEX;
		goto errors;
	}
	if (gpio_mode >= S2LP_GPIO_MODE_LAST) {
		status = S2LP_ERROR_GPIO_MODE;
		goto errors;
	}
	if ((gpio_function >= S2LP_GPIO_INPUT_FUNCTION_LAST) && (gpio_function >= S2LP_GPIO_OUTPUT_FUNCTION_LAST)) {
		status = S2LP_ERROR_GPIO_FUNCTION;
		goto errors;
	}
	if (fifo_flag_direction >= S2LP_FIFO_FLAG_DIRECTION_LAST) {
		status = S2LP_ERROR_FIFO_FLAG_DIRECTION;
		goto errors;
	}
	// Read corresponding register.
	status = S2LP_read_register((S2LP_REG_GPIO0_CONF + gpio_index), &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Set required bits.
	reg_value &= 0x04; // Bit 2 is reserved.
	reg_value |= ((gpio_mode & 0x02) << 0);
	reg_value |= ((gpio_function & 0x1F) << 3);
	// Write register.
	status = S2LP_write_register((S2LP_REG_GPIO0_CONF + gpio_index), reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Select FIFO flags.
	status = S2LP_read_register(S2LP_REG_PROTOCOL2, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	reg_value &= 0xFB;
	reg_value |= ((fifo_flag_direction & 0x01) << 2);
	status = S2LP_write_register(S2LP_REG_PROTOCOL2, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET FIFO THRESHOLDS.
 * @param fifo_threshold:	FIFO threshold to set.
 * @param threshold_value:	Threshold value (number of bytes).
 * @return status:			Function execution status.
 */
S2LP_status_t S2LP_set_fifo_threshold(S2LP_fifo_threshold_t fifo_threshold, unsigned char threshold_value) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	// Check parameters.
	if (fifo_threshold >= S2LP_FIFO_THRESHOLD_LAST) {
		status = S2LP_ERROR_FIFO_THRESHOLD;
		goto errors;
	}
	if (threshold_value > S2LP_FIFO_THHRESHOLD_BYTES_MAX) {
		status = S2LP_ERROR_FIFO_THRESHOLD_VALUE;
		goto errors;
	}
	// Write register.
	status = S2LP_write_register(fifo_threshold, threshold_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* CONFIGURE S2LP INTERRUPT.
 * @param irq_idx:		Interrupt index.
 * @param irq_enable:	Enable (1) or disable (0) interrupt.
 * @return status:		Function execution status.
 */
S2LP_status_t S2LP_configure_irq(S2LP_irq_index_t irq_idx, unsigned char irq_enable) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	unsigned char reg_addr_offset = 0;
	unsigned char irq_bit_offset = 0;
	// Check parameter.
	if (irq_idx >= S2LP_IRQ_INDEX_LAST) {
		status = S2LP_ERROR_IRQ_INDEX;
		goto errors;
	}
	// Get register and bit offsets.
	reg_addr_offset = (irq_idx / 8);
	irq_bit_offset = (irq_idx % 8);
	// Read register.
	status = S2LP_read_register((S2LP_REG_IRQ_MASK0 - reg_addr_offset), &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Set bit.
	reg_value &= ~(0b1 << irq_bit_offset);
	reg_value |= (((irq_enable == 0) ? 0b1 : 0b0) << irq_bit_offset);
	// Program register.
	status = S2LP_write_register((S2LP_REG_IRQ_MASK0 - reg_addr_offset), reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* CLEAR S2LP IRQ FLAGS.
 * @param:			None.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_clear_irq_flags(void) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	// Read IRQ status to clear flags.
	status = S2LP_read_register(S2LP_REG_IRQ_STATUS3,  &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_read_register(S2LP_REG_IRQ_STATUS2,  &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_read_register(S2LP_REG_IRQ_STATUS1,  &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_read_register(S2LP_REG_IRQ_STATUS0,  &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET PACKET LENGTH.
 * @param packet_length_bytes:	Packet length in bytes.
 * @return status:				Function execution status.
 */
S2LP_status_t S2LP_set_packet_length(unsigned char packet_length_bytes) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	// Set length.
	status = S2LP_write_register(S2LP_REG_PCKTLEN1, 0x00);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_write_register(S2LP_REG_PCKTLEN0, packet_length_bytes);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET RX PREAMBLE DETECTOR LENGTH.
 * @param preamble_length_bytes:	Number of '01' or '10' patterns of the preamble.
 * @param preamble_polarity:		Preamble polarity (0/1).
 * @return status:					Function execution status.
 */
S2LP_status_t S2LP_set_preamble_detector(unsigned char preamble_length_2bits, S2LP_preamble_pattern_t preamble_pattern) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char pcktctrlx_reg_value = 0;
	// Check parameter.
	if (preamble_pattern >= S2LP_PREAMBLE_PATTERN_LAST) {
		status = S2LP_ERROR_PREAMBLE_PATTERN;
		goto errors;
	}
	// Set length.
	status = S2LP_read_register(S2LP_REG_PCKTCTRL6, &pcktctrlx_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	pcktctrlx_reg_value &= 0xFC;
	status = S2LP_write_register(S2LP_REG_PCKTCTRL6, pcktctrlx_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	status = S2LP_write_register(S2LP_REG_PCKTCTRL5, preamble_length_2bits);
	if (status != S2LP_SUCCESS) goto errors;
	// Set pattern.
	status = S2LP_read_register(S2LP_REG_PCKTCTRL3, &pcktctrlx_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	pcktctrlx_reg_value &= 0xFC;
	pcktctrlx_reg_value |= (preamble_pattern & 0x03);
	status = S2LP_write_register(S2LP_REG_PCKTCTRL3, pcktctrlx_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* CONFIGURE RX SYNC WORD DETECTOR.
 * @param sync_word:				Byte array containing the synchronization word.
 * @param sync_word_length_bits:	Length of the synchronization word in bits.
 * @return status:					Function execution status.
 */
S2LP_status_t S2LP_set_sync_word(unsigned char* sync_word, unsigned char sync_word_length_bits) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char sync_word_length_bytes = 0;
	unsigned char generic_byte = 0;
	// Check parameters.
	if (sync_word == (void*) 0) {
		status = S2LP_ERROR_SYNC_WORD;
		goto errors;
	}
	if (sync_word_length_bits > S2LP_SYNC_WORD_LENGTH_BITS_MAX) {
		status = S2LP_ERROR_SYNC_WORD_LENGTH;
		goto errors;
	}
	// Set synchronization word.
	sync_word_length_bytes = (sync_word_length_bits / 8);
	if ((sync_word_length_bits - (sync_word_length_bytes * 8)) > 0) {
		sync_word_length_bytes++;
	}
	for (generic_byte=0 ; generic_byte<sync_word_length_bytes ; generic_byte++) {
		status = S2LP_write_register((S2LP_REG_SYNC0 - generic_byte), sync_word[generic_byte]);
		if (status != S2LP_SUCCESS) goto errors;
	}
	// Set length.
	status = S2LP_read_register(S2LP_REG_PCKTCTRL6, &generic_byte);
	if (status != S2LP_SUCCESS) goto errors;
	generic_byte &= 0x03;
	generic_byte |= (sync_word_length_bits << 2);
	status = S2LP_write_register(S2LP_REG_PCKTCTRL6, generic_byte);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* DISABLE CRC.
 * @param:			None.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_disable_crc(void) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	// Read register.
	status = S2LP_read_register(S2LP_REG_PCKTCTRL1, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Set bits.
	reg_value &= 0x1F;
	// Write register.
	status = S2LP_write_register(S2LP_REG_PCKTCTRL1, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* CONFIGURE TX POWER AMPLIFIER.
 * @param:			None.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_configure_pa(void) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	// Disable PA power ramping and select slot 0.
	status = S2LP_write_register(S2LP_REG_PA_POWER0, 0x00);
	if (status != S2LP_SUCCESS) goto errors;
	// Disable FIR.
	status = S2LP_read_register(S2LP_REG_PA_CONFIG1, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	reg_value &= 0xFD;
	status = S2LP_write_register(S2LP_REG_PA_CONFIG1, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Enable interpolator.
	S2LP_read_register(S2LP_REG_MOD1, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	reg_value |= 0x80;
	status = S2LP_write_register(S2LP_REG_MOD1, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* CONFIGURE TX OUTPUT POWER.
 * @param output_power_dbm:	RF output power in dBm.
 * @return status:			Function execution status.
 */
S2LP_status_t S2LP_set_rf_output_power(signed char output_power_dbm) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	unsigned char pa_reg_value = 0;
	// Check parameter.
	if (output_power_dbm > S2LP_RF_OUTPUT_POWER_MAX) {
		status = S2LP_ERROR_RF_OUTPUT_POWER_OVERFLOW;
		goto errors;
	}
	if (output_power_dbm < S2LP_RF_OUTPUT_POWER_MIN) {
		status = S2LP_ERROR_RF_OUTPUT_POWER_UNDERFLOW;
		goto errors;
	}
	// Compute register value.
	pa_reg_value = (unsigned char) (29 - 2 * output_power_dbm);
	// Program register.
	status = S2LP_read_register(S2LP_REG_PA_POWER1, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	reg_value &= 0x80;
	reg_value |= (pa_reg_value & 0x7F);
	status = S2LP_write_register(S2LP_REG_PA_POWER1, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET S2LP TX DATA SOURCE.
 * @param tx_source:	TX data source (use enumeration defined in s2lp.h).
 * @return status:		Function execution status.
 */
S2LP_status_t S2LP_set_tx_source(S2LP_tx_source_t tx_source) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	// Check parameter.
	if (tx_source >= S2LP_TX_SOURCE_LAST) {
		status = S2LP_ERROR_TX_SOURCE;
		goto errors;
	}
	// Read register.
	status = S2LP_read_register(S2LP_REG_PCKTCTRL1, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Set bits.
	reg_value &= 0xF3;
	reg_value |= (tx_source << 2);
	// Write register.
	status = S2LP_write_register(S2LP_REG_PCKTCTRL1, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* INITIATE FIFO WRITING OPERATION.
 * @param tx_data:				Data to sent to FIFO.
 * @param tx_data_length_bytes:	Number of bytes to send.
 * @return status:				Function execution status.
 */
S2LP_status_t S2LP_write_fifo(unsigned char* tx_data, unsigned char tx_data_length_bytes) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
#ifndef S2LP_TX_FIFO_USE_DMA
	unsigned char idx = 0;
#endif
	// Check parameters.
	if (tx_data == (void*) 0) {
		status = S2LP_ERROR_TX_DATA;
		goto errors;
	}
	if (tx_data_length_bytes > S2LP_FIFO_SIZE_BYTES) {
		status = S2LP_ERROR_TX_DATA_LENGTH;
		goto errors;
	}
#ifdef S2LP_TX_FIFO_USE_DMA
	// Set buffer address.
	DMA1_set_channel3_source_addr((unsigned int) tx_data, tx_data_length_bytes);
#endif
	// Falling edge on CS pin.
	GPIO_write(&GPIO_S2LP_CS, 0);
	// Access FIFO.
	spi_status = SPI1_write_byte(S2LP_HEADER_BYTE_WRITE); // A/C='1' and W/R='0'.
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
	spi_status = SPI1_write_byte(S2LP_REG_FIFO);
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
#ifdef S2LP_TX_FIFO_USE_DMA
	// Transfer buffer with DMA.
	DMA1_start_channel3();
	while (DMA1_get_channel3_status() == 0) {
		PWR_enter_sleep_mode();
	}
	DMA1_stop_channel3();
#else
	for (idx=0 ; idx<tx_data_length_bytes ; idx++) {
		spi_status = SPI1_write_byte(tx_data[idx]);
		SPI1_status_check(S2LP_ERROR_BASE_SPI);
	}
#endif
errors:
	GPIO_write(&GPIO_S2LP_CS, 1); // Set CS pin.
	return status;
}

/* SET S2LP RX SOURCE.
 * @param rx_source:	RX data source.
 * @return status:		Function execution status.
 */
S2LP_status_t S2LP_set_rx_source(S2LP_rx_source_t rx_source) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char reg_value = 0;
	// Check parameter.
	if (rx_source >= S2LP_RX_SOURCE_LAST) {
		status = S2LP_ERROR_RX_SOURCE;
		goto errors;
	}
	// Read register.
	status = S2LP_read_register(S2LP_REG_PCKTCTRL3, &reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Set bits.
	reg_value &= 0xCF;
	reg_value |= (rx_source << 4);
	// Write register.
	status = S2LP_write_register(S2LP_REG_PCKTCTRL3, reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* SET RX FILTER BANDWIDTH.
 * @param rxbw_setting:	RX bandwidth mantissa and exponent setting.
 * @return status:		Function execution status.
 */
S2LP_status_t S2LP_set_rx_bandwidth(S2LP_mantissa_exponent_t rxbw_setting) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char chflt_reg_value = 0;
	// Write register.
	chflt_reg_value = ((rxbw_setting.mantissa << 4) & 0xF0) + (rxbw_setting.exponent & 0x0F);
	status = S2LP_write_register(S2LP_REG_CHFLT, chflt_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* DISABLE CS, EQUALIZATION AND ANTENNA SWITCHING.
 * @param:			None.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_disable_equa_cs_ant_switch(void) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char ant_select_conf_reg_value = 0;
	// Read register.
	status = S2LP_read_register(S2LP_REG_ANT_SELECT_CONF, &ant_select_conf_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Disable equalization.
	ant_select_conf_reg_value &= 0x83;
	// Program register.
	status = S2LP_write_register(S2LP_REG_ANT_SELECT_CONF, ant_select_conf_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
errors:
	return status;
}

/* GET CURRENT RSSI LEVEL.
 * @param rssi_dbm:	Pointer that will contain RSSI in dBm.
 * @return status:	Function execution status.
 */
S2LP_status_t S2LP_get_rssi(signed short* rssi_dbm) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	unsigned char rssi_level_reg_value = 0;
	// Read register.
	status = S2LP_read_register(S2LP_REG_RSSI_LEVEL, &rssi_level_reg_value);
	if (status != S2LP_SUCCESS) goto errors;
	// Convert to dBm.
	(*rssi_dbm) = (signed short) rssi_level_reg_value - (signed short) S2LP_RSSI_OFFSET_DB;
errors:
	return status;
}

/* READ S2LP RX FIFO.
 * @param rx_data:				Byte array that will contain FIFO data.
 * @param rx_data_length_bytes:	Number of bytes to read.
 * @return status:				Function execution status.
 */
S2LP_status_t S2LP_read_fifo(unsigned char* rx_data, unsigned char rx_data_length_bytes) {
	// Local variables.
	S2LP_status_t status = S2LP_SUCCESS;
	SPI_status_t spi_status = SPI_SUCCESS;
	unsigned char idx = 0;
	// Check parameters.
	if (rx_data == (void*) 0) {
		status = S2LP_ERROR_RX_DATA;
		goto errors;
	}
	if (rx_data_length_bytes > S2LP_FIFO_SIZE_BYTES) {
		status = S2LP_ERROR_RX_DATA_LENGTH;
		goto errors;
	}
	// Falling edge on CS pin.
	GPIO_write(&GPIO_S2LP_CS, 0);
	// Burst read sequence.
	spi_status = SPI1_write_byte(S2LP_HEADER_BYTE_READ); // A/C='0' and W/R='1'.
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
	spi_status = SPI1_write_byte(S2LP_REG_FIFO);
	SPI1_status_check(S2LP_ERROR_BASE_SPI);
	for (idx=0 ; idx<rx_data_length_bytes ; idx++) {
		spi_status = SPI1_read_byte(0xFF, &(rx_data[idx]));
		SPI1_status_check(S2LP_ERROR_BASE_SPI);
	}
errors:
	GPIO_write(&GPIO_S2LP_CS, 1); // Set CS pin.
	return status;
}
