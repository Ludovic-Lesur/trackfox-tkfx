/*
 * s2lp.c
 *
 *  Created on: 12 oct. 2019
 *      Author: Ludo
 */

#include "s2lp.h"

#include "gpio.h"
#include "mapping.h"
#include "s2lp_reg.h"
#include "spi.h"

/*** S2LP local macros ***/

#define S2LP_HEADER_BYTE_WRITE			0x00
#define S2LP_HEADER_BYTE_READ			0x01
#define S2LP_HEADER_BYTE_COMMAND		0x80
#define S2LP_XO_FREQUENCY_HZ			26000000
#define S2LP_SYNC_WORD_LENGTH_BITS_MAX	32
#define S2LP_RSSI_OFFSET_DB				146
#define S2LP_FIFO_SIZE_BYTES			128
#define S2LP_RF_OUTPUT_POWER_MIN		-30
#define S2LP_RF_OUTPUT_POWER_MAX		14

/*** S2LP local functions ***/

/* S2LP REGISTER WRITE FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param valie:	Value to write in register.
 * @return:			None.
 */
void S2LP_WriteRegister(unsigned char addr, unsigned char value) {
	// Falling edge on CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 0);
	// Write sequence.
	SPI1_WriteByte(S2LP_HEADER_BYTE_WRITE); // A/C='0' and W/R='0'.
	SPI1_WriteByte(addr);
	SPI1_WriteByte(value);
	// Set CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 1);
}

/* S2LP REGISTER READ FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param value:	Pointer to byte that will contain the register Value to read.
 * @return:			None.
 */
void S2LP_ReadRegister(unsigned char addr, unsigned char* value) {
	// Falling edge on CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 0);
	// Read sequence.
	SPI1_WriteByte(S2LP_HEADER_BYTE_READ); // A/C='0' and W/R='1'.
	SPI1_WriteByte(addr);
	SPI1_ReadByte(0xFF, value);
	// Set CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 1);
}

/*** S2LP functions ***/

/* INIT S2LP TRANSCEIVER.
 * @param:	None.
 * @return:	None.
 */
void S2LP_Init(void) {
	// Configure GPIOs.
	GPIO_Configure(&GPIO_S2LP_GPIO0, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_S2LP_GPIO3, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* DISABLE S2LP GPIOs.
 * @param:	None.
 * @return:	None.
 */
void S2LP_DisableGpio(void) {
	// Configure GPIOs as analog inputs.
	GPIO_Configure(&GPIO_S2LP_GPIO0, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_S2LP_GPIO3, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* SEND COMMAND TO S2LP.
 * @param command:	Command to send (use enum defined in s2lp_reg.h).
 * @return:			None.
 */
void S2LP_SendCommand(S2LP_Command command) {
	// Falling edge on CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 0);
	// Write sequence.
	SPI1_WriteByte(S2LP_HEADER_BYTE_COMMAND); // A/C='1' and W/R='0'.
	SPI1_WriteByte(command);
	// Set CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 1);
}

/* CONFIGURE PLL CHARGE-PUMP.
 * @param:	None.
 * @return:	None.
 */
void S2LP_ConfigureChargePump(void) {
	// PLL_CP_ISEL is '010' by default.
	// For 26MHz oscillator, current pulses must be increased (PLL_PFD_SPLIT_EN='1').
	unsigned char synth_config2_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_SYNTH_CONFIG2, &synth_config2_reg_value);
	// Set bit.
	synth_config2_reg_value |= (1 << 2);
	// Write register.
	S2LP_WriteRegister(S2LP_REG_SYNTH_CONFIG2, synth_config2_reg_value);
}

/* SET S2LP MODULATION SCHEME.
 * @param modulation:	Selected modulation (use enum defined in s2lp.h).
 * @return:				None.
 */
void S2LP_SetModulation(S2LP_Modulation modulation) {
	// Read register.
	unsigned char mod2_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_MOD2, &mod2_reg_value);
	// Change required bits.
	mod2_reg_value &= 0x0F;
	mod2_reg_value |= (modulation << 4);
	// Write register.
	S2LP_WriteRegister(S2LP_REG_MOD2, mod2_reg_value);
}

/* SET TRANSCEIVER RF CENTRAL FREQUENCY.
 * @param rf_frequency_hz:	RF frequency in Hz.
 * @return:					None.
 */
void S2LP_SetRfFrequency(unsigned int rf_frequency_hz) {
	// See equation p.27 of S2LP datasheet.
	// BS (band select), CHNUM and REFDIV are 0 by default (high band, D=1).
	// B=4 for 868MHz (high band).
	// SYNT = (fRF * 2^20 * B/2 * D) / (fXO) = (fRF * 2^21) / (fXO).
	unsigned long long synt_value = 0b1 << 21;
	synt_value *= rf_frequency_hz;
	synt_value /= S2LP_XO_FREQUENCY_HZ;
	// Write registers.
	unsigned char synt_reg_value = ((synt_value >> 24) & 0x0F) | 0x40;
	S2LP_WriteRegister(S2LP_REG_SYNT3, synt_reg_value);
	synt_reg_value = (synt_value >> 16) & 0xFF;
	S2LP_WriteRegister(S2LP_REG_SYNT2, synt_reg_value);
	synt_reg_value = (synt_value >> 8) & 0xFF;
	S2LP_WriteRegister(S2LP_REG_SYNT1, synt_reg_value);
	synt_reg_value = (synt_value >> 0) & 0xFF;
	S2LP_WriteRegister(S2LP_REG_SYNT0, synt_reg_value);
}

/* SET FSK DEVIATION.
 * @param fsk_deviation_setting:	FSK deviation mantissa and exponent setting.
 * @return:							None.
 */
void S2LP_SetFskDeviation(S2LP_MantissaExponent* fsk_deviation_setting) {
	// Write registers.
	S2LP_WriteRegister(S2LP_REG_MOD0, (fsk_deviation_setting -> mantissa));
	unsigned char mod1_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_MOD1, &mod1_reg_value);
	mod1_reg_value &= 0xF0;
	mod1_reg_value |= (fsk_deviation_setting -> exponent);
	S2LP_WriteRegister(S2LP_REG_MOD1, mod1_reg_value);
}

/* SET DATA BIT RATE.
 * @param bit_rate_setting:	Bit rate FSK deviation mantissa and exponent setting.
 * @return:					None.
 */
void S2LP_SetBitRate(S2LP_MantissaExponent* bit_rate_setting) {
	// Write registers.
	S2LP_WriteRegister(S2LP_REG_MOD4, ((bit_rate_setting -> mantissa) >> 8) & 0x00FF);
	S2LP_WriteRegister(S2LP_REG_MOD3, ((bit_rate_setting -> mantissa) >> 0) & 0x00FF);
	unsigned char mod2_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_MOD2, &mod2_reg_value);
	mod2_reg_value &= 0xF0;
	mod2_reg_value |= (bit_rate_setting -> exponent);
	S2LP_WriteRegister(S2LP_REG_MOD2, mod2_reg_value);
}

/* CONFIGURE S2LP GPIOs.
 * @param gpio_number:		GPIO to configure (0 to 3).
 * @param gpio_mode:		GPIO mode (use enum defined in s2lp.h).
 * @param gpio_function:	GPIO function (use enum defined in s2lp.h).
 * @return:					None.
 */
void S2LP_ConfigureGpio(unsigned char gpio_number, S2LP_GPIO_Mode gpio_mode, unsigned char gpio_function) {
	// Read corresponding register.
	unsigned char gpiox_conf_reg_value = 0;
	S2LP_ReadRegister((S2LP_REG_GPIO0_CONF + gpio_number), &gpiox_conf_reg_value);
	// Set required bits.
	gpiox_conf_reg_value &= 0x04; // Bit 2 is reserved.
	gpiox_conf_reg_value |= ((gpio_mode & 0x02) << 0);
	gpiox_conf_reg_value |= ((gpio_function & 0x1F) << 3);
	// Write register.
	S2LP_WriteRegister((S2LP_REG_GPIO0_CONF + gpio_number), gpiox_conf_reg_value);
}

/* GET S2LP IRQ FLAGS.
 * @param:	None.
 * @return:	TBD.
 */
unsigned int S2LP_GetIrqFlags(void) {
	unsigned int irq_flags = 0;
	unsigned char irq_statusx_reg_value = 0;
	// Read interrupt registers.
	S2LP_ReadRegister(S2LP_REG_IRQ_STATUS3, &irq_statusx_reg_value);
	irq_flags |= (irq_statusx_reg_value << 24);
	S2LP_ReadRegister(S2LP_REG_IRQ_STATUS2, &irq_statusx_reg_value);
	irq_flags |= (irq_statusx_reg_value << 16);
	S2LP_ReadRegister(S2LP_REG_IRQ_STATUS1, &irq_statusx_reg_value);
	irq_flags |= (irq_statusx_reg_value << 8);
	S2LP_ReadRegister(S2LP_REG_IRQ_STATUS0, &irq_statusx_reg_value);
	irq_flags |= (irq_statusx_reg_value << 0);
	// Return status vector.
	return irq_flags;
}

/* SET TX RF OUTPUT POER.
 * @param rf_output_power_dbm:	RF output power in dBm.
 * @return:						None.
 */
void S2LP_SetRfOutputPower(signed char rf_output_power_dbm) {
	// Clamp value if needed.
	signed char local_rf_output_power_dbm = rf_output_power_dbm;
	if (local_rf_output_power_dbm < S2LP_RF_OUTPUT_POWER_MIN) {
		local_rf_output_power_dbm = S2LP_RF_OUTPUT_POWER_MIN;
	}
	if (local_rf_output_power_dbm > S2LP_RF_OUTPUT_POWER_MAX) {
		local_rf_output_power_dbm = S2LP_RF_OUTPUT_POWER_MAX;
	}
	// Program register.
	unsigned char pa_power_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_PA_POWER1, &pa_power_reg_value);
	pa_power_reg_value &= 0x80;
	pa_power_reg_value |= (2 * (local_rf_output_power_dbm - (S2LP_RF_OUTPUT_POWER_MIN))) & 0x3F;
	S2LP_WriteRegister(S2LP_REG_PA_POWER1, pa_power_reg_value);
}

/* SET RX FILTER BANDWIDTH.
 * @param bit_rate_setting:	RX bandwidth mantissa and exponent setting.
 * @return:					None.
 */
void S2LP_SetRxBandwidth(S2LP_MantissaExponent* rxbw_setting) {
	// Write register.
	unsigned char chflt_reg_value = (((rxbw_setting -> mantissa) << 4) & 0xF0) + ((rxbw_setting -> exponent) & 0x0F);
	S2LP_WriteRegister(S2LP_REG_CHFLT, chflt_reg_value);
}

/* SET RX PREAMBLE DETECTOR LENGTH.
 * @param preamble_length_bytes:	Preamble length in bytes.
 * @param preamble_polarity:		Preamble polarity (0/1).
 * @return:							None.
 */
void S2LP_SetPreambleDetector(unsigned char preamble_length_bytes, S2LP_PreamblePattern preamble_pattern) {
	// Set length.
	unsigned char pcktctrlx_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_PCKTCTRL6, &pcktctrlx_reg_value);
	pcktctrlx_reg_value &= 0xFC;
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL6, pcktctrlx_reg_value);
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL5, preamble_length_bytes);
	// Set pattern.
	S2LP_ReadRegister(S2LP_REG_PCKTCTRL3, &pcktctrlx_reg_value);
	pcktctrlx_reg_value &= 0xFC;
	pcktctrlx_reg_value |= (preamble_pattern & 0x03);
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL3, pcktctrlx_reg_value);
}

/* CONFIGURE RX SYNC WORD DETECTOR.
 * @param sync_word:				Byte array containing the synchronization word.
 * @param sync_word_length_bits:	Length of the synchronization word in bits.
 * @return:							None.
 */
void S2LP_SetSyncWord(unsigned char* sync_word, unsigned char sync_word_length_bits) {
	// Clamp value if needed.
	unsigned char local_sync_word_length_bits = sync_word_length_bits;
	if (local_sync_word_length_bits > S2LP_SYNC_WORD_LENGTH_BITS_MAX) {
		local_sync_word_length_bits = S2LP_SYNC_WORD_LENGTH_BITS_MAX;
	}
	// Set synchronization word.
	unsigned char sync_word_length_bytes = (local_sync_word_length_bits / 8);
	if ((local_sync_word_length_bits - (sync_word_length_bytes * 8)) > 0) {
		sync_word_length_bytes++;
	}
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<sync_word_length_bytes ; byte_idx++) {
		S2LP_WriteRegister((S2LP_REG_SYNC0 - byte_idx), sync_word[byte_idx]);
	}
	// Set length.
	unsigned char pcktctrl6_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_PCKTCTRL6, &pcktctrl6_reg_value);
	pcktctrl6_reg_value &= 0x03;
	pcktctrl6_reg_value |= (local_sync_word_length_bits << 2);
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL6, pcktctrl6_reg_value);
}

/* CONFIGURE RX DATA LENGTH.
 * @param rx_data_length_bytes:	RX data packet length in bytes.
 * @return:						None.
 */
void S2LP_SetRxDataLength(unsigned char rx_data_length_bytes) {
	// Set RX FIFO full threshold to the expected number of bytes.
	unsigned char fifo_configx_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_FIFO_CONFIG3, &fifo_configx_reg_value);
	fifo_configx_reg_value &= 0x80;
	fifo_configx_reg_value |= (rx_data_length_bytes & 0x7F);
	S2LP_WriteRegister(S2LP_REG_FIFO_CONFIG3, fifo_configx_reg_value);
	// Set RX FIFO empty threshold to 0.
	S2LP_ReadRegister(S2LP_REG_FIFO_CONFIG2, &fifo_configx_reg_value);
	fifo_configx_reg_value &= 0x80;
	S2LP_WriteRegister(S2LP_REG_FIFO_CONFIG2, fifo_configx_reg_value);
}

/* GET CURRENT RSSI LEVEL.
 * @param:		None.
 * return rssi:	Current RSSI level in dBm (at sync).
 */
signed char S2LP_GetRssi(void) {
	unsigned char rssi_level_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_RSSI_LEVEL, &rssi_level_reg_value);
	signed char rssi = rssi_level_reg_value - S2LP_RSSI_OFFSET_DB;
	return rssi;
}

/* READ S2LP RX FIFO.
 * @param rx_data:				Byte array that will contain FIFO data.
 * @param rx_data_length_bytes:	Number of bytes to read.
 */
void S2LP_ReadRxFifo(unsigned char* rx_data, unsigned char rx_data_length_bytes) {
	// Clamp value if needed.
	unsigned char local_rx_data_length_bytes = rx_data_length_bytes;
	if (local_rx_data_length_bytes > S2LP_FIFO_SIZE_BYTES) {
		local_rx_data_length_bytes = S2LP_FIFO_SIZE_BYTES;
	}
	// Falling edge on CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 0);
	// Burst read sequence.
	SPI1_WriteByte(S2LP_HEADER_BYTE_READ); // A/C='0' and W/R='1'.
	SPI1_WriteByte(S2LP_REG_FIFO);
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<local_rx_data_length_bytes ; byte_idx++) {
		SPI1_ReadByte(0xFF, &(rx_data[byte_idx]));
	}
	// Set CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 1);
}
