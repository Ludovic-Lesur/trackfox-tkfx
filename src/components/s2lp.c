/*
 * s2lp.c
 *
 *  Created on: 12 oct. 2019
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

#define S2LP_HEADER_BYTE_WRITE				0x00
#define S2LP_HEADER_BYTE_READ				0x01
#define S2LP_HEADER_BYTE_COMMAND			0x80

#define S2LP_XO_FREQUENCY_HZ				26000000
#define S2LP_XO_HIGH_RANGE_THRESHOLD_HZ		48000000

#define S2LP_SYNC_WORD_LENGTH_BITS_MAX		32
#define S2LP_RSSI_OFFSET_DB					146
#define S2LP_RF_OUTPUT_POWER_MIN			-30
#define S2LP_RF_OUTPUT_POWER_MAX			14

#define S2LP_TX_FIFO_USE_DMA // Use DMA to fill TX FIFO, standard SPI access otherwise.

/*** S2LP local functions ***/

/* S2LP REGISTER WRITE FUNCTION.
 * @param addr:		Register address (7 bits).
 * @param valie:	Value to write in register.
 * @return:			None.
 */
static void S2LP_WriteRegister(unsigned char addr, unsigned char value) {
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
static void S2LP_ReadRegister(unsigned char addr, unsigned char* value) {
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

/* INIT S2LP INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void S2LP_Init(void) {
	// Configure TCXO power control pin.
	GPIO_Configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_TCXO_POWER_ENABLE, 0);
}

/* DISABLE S2LP INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void S2LP_Disable(void) {
	// Configure GPIOs as analog inputs.
	GPIO_Configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_S2LP_GPIO0, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef HW1_1
	GPIO_Configure(&GPIO_S2LP_SDN, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}

/* CONFIGURE S2LP GPIO0.
 * @param pull_resistor_config:	0 for pull-down, pull_up otherwise.
 * @return:						None.
 */
void S2LP_SetGpio0(unsigned char pull_resistor_config) {
	if (pull_resistor_config == 0) {
		// Uplink configuration (pull-down + rising edge).
		GPIO_Configure(&GPIO_S2LP_GPIO0, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
		EXTI_ConfigureGpio(&GPIO_S2LP_GPIO0, EXTI_TRIGGER_RISING_EDGE);
	}
	else {
		// Downlink configuration -pull_up + falling edge).
		GPIO_Configure(&GPIO_S2LP_GPIO0, GPIO_MODE_INPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_UP);
		EXTI_ConfigureGpio(&GPIO_S2LP_GPIO0, EXTI_TRIGGER_FALLING_EDGE);
	}
}

/* CONTROL EXTERNAL TCXO.
 * @param:	None.
 * @return:	None.
 */
void S2LP_Tcxo(unsigned char tcxo_enable) {
	// Turn TCXO on or off.
	if (tcxo_enable != 0) {
		GPIO_Write(&GPIO_TCXO_POWER_ENABLE, 1);
		LPTIM1_DelayMilliseconds(100, 1);
	}
	else {
		GPIO_Write(&GPIO_TCXO_POWER_ENABLE, 0);
	}
}

/* PUT S2LP IN SHUTDOWN MODE.
 * @param:	None.
 * @return:	None.
 */
void S2LP_EnterShutdown(void) {
	// Put SDN in high impedance (pull-up resistor used).
#ifdef HW1_1
	GPIO_Configure(&GPIO_S2LP_SDN, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
}

/* PUT S2LP IN ACTIVE MODE.
 * @param:	None.
 * @return:	None.
 */
void S2LP_ExitShutdown(void) {
	// Put SDN low.
#ifdef HW1_1
	GPIO_Configure(&GPIO_S2LP_SDN, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_S2LP_SDN, 0);
#endif
	// Wait for reset time.
	LPTIM1_DelayMilliseconds(100, 1);
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

/* WAIT FOR S2LP TO ENTER A GIVEN STATE.
 * @param new_state:	State to reach.
 * @return:				None.
 */
void S2LP_WaitForStateSwitch(S2LP_State new_state) {
	unsigned char state = 0;
	unsigned char reg_value = 0;
	// Poll MC_STATE until state is reached.
	do {
		S2LP_ReadRegister(S2LP_REG_MC_STATE0, &reg_value);
		state = (reg_value >> 1) & 0x7F;
	}
	while (state != new_state);
}

/* WAIT FOR S2LP OSCILLATOR TO BE RUNNING.
 * @param:	None.
 * @return:	None.
 */
void S2LP_WaitForXo(void) {
	unsigned char xo_on = 0;
	unsigned char reg_value = 0;
	// Poll MC_STATE until state is reached.
	do {
		S2LP_ReadRegister(S2LP_REG_MC_STATE0, &reg_value);
		xo_on = (reg_value & 0x01);
	}
	while (xo_on == 0);
}

/* CONFIGURE S2LP OSCILLATOR.
 * @param s2lp_oscillator:	S2LP oscillator type (use enumeration defined in s2lp.h).
 * @return:					None.
 */
void S2LP_SetOscillator(S2LP_Oscillator s2lp_oscillator) {
	// Set RFDIV to 0, disable external RCO, configure EXT_REF bit.
	unsigned char reg_value = (s2lp_oscillator == S2LP_OSCILLATOR_TCXO) ? 0xB0 : 0x30;
	S2LP_WriteRegister(S2LP_REG_XO_RCO_CONF0, reg_value);
	// Set digital clock divider according to crytal frequency.
	reg_value = (S2LP_XO_FREQUENCY_HZ < S2LP_XO_HIGH_RANGE_THRESHOLD_HZ) ? 0x3E : 0x2E;
	// Write register.
	S2LP_WriteRegister(S2LP_REG_XO_RCO_CONF1, reg_value);
}

/* ENABLE INTERNAL DC-DC REGULATOR (SMPS).
 * @param:	None.
 * @return:	None.
 */
void S2LP_ConfigureSmps(S2LP_SmpsSetting smps_setting) {
	// Configure divider and switching frequency.
	S2LP_WriteRegister(S2LP_REG_PM_CONF3, smps_setting.s2lp_smps_reg_pm_conf3);
	S2LP_WriteRegister(S2LP_REG_PM_CONF2, smps_setting.s2lp_smps_reg_pm_conf2);
}

/* CONFIGURE PLL CHARGE-PUMP.
 * @param:	None.
 * @return:	None.
 */
void S2LP_ConfigureChargePump(void) {
	// Set PLL_CP_ISEL to '010'.
	unsigned char reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_SYNT3, &reg_value);
	// Set bits.
	reg_value &= 0x1F;
	reg_value |= (0b010 << 5);
	// Write register.
	S2LP_WriteRegister(S2LP_REG_SYNT3, reg_value);
	// Set PLL_PFD_SPLIT_EN bit according to crystal frequency.
	S2LP_ReadRegister(S2LP_REG_SYNTH_CONFIG2, &reg_value);
	if (S2LP_XO_FREQUENCY_HZ >= S2LP_XO_HIGH_RANGE_THRESHOLD_HZ) {
		reg_value &= 0xFB;
	}
	else {
		reg_value |= 0x04;
	}
	// Write register.
	S2LP_WriteRegister(S2LP_REG_SYNTH_CONFIG2, reg_value);
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
	// Set IF to 300kHz.
	if (S2LP_XO_FREQUENCY_HZ < S2LP_XO_HIGH_RANGE_THRESHOLD_HZ) {
		S2LP_WriteRegister(S2LP_REG_IF_OFFSET_ANA, 0xB8);
	}
	// See equation p.27 of S2LP datasheet.
	// Set CHNUM to 0.
	S2LP_WriteRegister(S2LP_REG_CHNUM, 0x00);
	// B=4 for 868MHz (high band, BS=0). REFDIV was set to 0 in oscillator configuration function.
	// SYNT = (fRF * 2^20 * B/2 * D) / (fXO) = (fRF * 2^21) / (fXO).
	unsigned long long synt_value = 0b1 << 21;
	synt_value *= rf_frequency_hz;
	synt_value /= S2LP_XO_FREQUENCY_HZ;
	// Write registers.
	unsigned char synt_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_SYNT3, &synt_reg_value);
	synt_reg_value &= 0xE0; // BS=0 to select high band.
	synt_reg_value |= ((synt_value >> 24) & 0x0F);
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
void S2LP_SetFskDeviation(S2LP_MantissaExponent fsk_deviation_setting) {
	// Write registers.
	S2LP_WriteRegister(S2LP_REG_MOD0, fsk_deviation_setting.mantissa);
	unsigned char mod1_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_MOD1, &mod1_reg_value);
	mod1_reg_value &= 0xF0;
	mod1_reg_value |= fsk_deviation_setting.exponent;
	S2LP_WriteRegister(S2LP_REG_MOD1, mod1_reg_value);
}

/* SET DATA BIT RATE.
 * @param bit_rate_setting:	Bit rate FSK deviation mantissa and exponent setting.
 * @return:					None.
 */
void S2LP_SetBitRate(S2LP_MantissaExponent bit_rate_setting) {
	// Write registers.
	S2LP_WriteRegister(S2LP_REG_MOD4, (bit_rate_setting.mantissa >> 8) & 0x00FF);
	S2LP_WriteRegister(S2LP_REG_MOD3, (bit_rate_setting.mantissa >> 0) & 0x00FF);
	unsigned char mod2_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_MOD2, &mod2_reg_value);
	mod2_reg_value &= 0xF0;
	mod2_reg_value |= (bit_rate_setting.exponent);
	S2LP_WriteRegister(S2LP_REG_MOD2, mod2_reg_value);
}

/* CONFIGURE S2LP GPIOs.
 * @param gpio_number:			GPIO to configure (0 to 3).
 * @param gpio_mode:			GPIO mode (use enum defined in s2lp.h).
 * @param gpio_function:		GPIO function (use enum defined in s2lp.h).
 * @param fifo_flag_direction:	'1' to select RX FIFO flags, '0' to select TX FIFO flags.
 * @return:						None.
 */
void S2LP_ConfigureGpio(unsigned char gpio_number, S2LP_GPIO_Mode gpio_mode, unsigned char gpio_function, unsigned char fifo_flag_direction) {
	// Read corresponding register.
	unsigned char reg_value = 0;
	S2LP_ReadRegister((S2LP_REG_GPIO0_CONF + gpio_number), &reg_value);
	// Set required bits.
	reg_value &= 0x04; // Bit 2 is reserved.
	reg_value |= ((gpio_mode & 0x02) << 0);
	reg_value |= ((gpio_function & 0x1F) << 3);
	// Write register.
	S2LP_WriteRegister((S2LP_REG_GPIO0_CONF + gpio_number), reg_value);
	// Select FIFO flags.
	S2LP_ReadRegister(S2LP_REG_PROTOCOL2, &reg_value);
	reg_value &= 0xFB;
	reg_value |= ((fifo_flag_direction & 0x01) << 2);
	S2LP_WriteRegister(S2LP_REG_PROTOCOL2, reg_value);
}

/* SET FIFO THRESHOLDS.
 * @param fifo_threshold:	FIFO threshold to set (use enumeration defined in s2lp.h).
 * @param threshold_value:	Threshold value (number of bytes).
 * @return:					None.
 */
void S2LP_SetFifoThreshold(S2LP_FifoThreshold fifo_threshold, unsigned char threshold_value) {
	// Write register.
	S2LP_WriteRegister(fifo_threshold, threshold_value);
}

/* CONFIGURE S2LP INTERRUPT.
 * @param irq_idx:		Interrupt index (use enumeration defined in s2lp.h).
 * @param irq_enable:	Enable (1) or disable (0) interrupt.
 * @return:				None.
 */
void S2LP_ConfigureIrq(S2LP_IrqIndex irq_idx, unsigned irq_enable) {
	// Get register and bit offsets.
	unsigned char reg_addr_offset = (irq_idx / 8);
	unsigned char irq_bit_offset = (irq_idx % 8);
	// Read register.
	unsigned char reg_value = 0;
	S2LP_ReadRegister((S2LP_REG_IRQ_MASK0 - reg_addr_offset), &reg_value);
	// Set bit.
	reg_value &= ~(0b1 << irq_bit_offset);
	reg_value |= (irq_enable << irq_bit_offset);
	// Program register.
	S2LP_WriteRegister((S2LP_REG_IRQ_MASK0 - reg_addr_offset), reg_value);
}

/* CLEAR S2LP IRQ FLAGS.
 * @param:	None.
 * @return:	None.
 */
void S2LP_ClearIrqFlags(void) {
	unsigned char reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_IRQ_STATUS3,  &reg_value);
	S2LP_ReadRegister(S2LP_REG_IRQ_STATUS2,  &reg_value);
	S2LP_ReadRegister(S2LP_REG_IRQ_STATUS1,  &reg_value);
	S2LP_ReadRegister(S2LP_REG_IRQ_STATUS0,  &reg_value);
}

/* SET PACKET LENGTH.
 * @param packet_length_bytes:	Packet length in bytes.
 * @return:						None.
 */
void S2LP_SetPacketlength(unsigned char packet_length_bytes) {
	// Set length.
	S2LP_WriteRegister(S2LP_REG_PCKTLEN1, 0x00);
	S2LP_WriteRegister(S2LP_REG_PCKTLEN0, packet_length_bytes);
}

/* SET RX PREAMBLE DETECTOR LENGTH.
 * @param preamble_length_bytes:	Number of '01' or '10' patterns of the preamble.
 * @param preamble_polarity:		Preamble polarity (0/1).
 * @return:							None.
 */
void S2LP_SetPreambleDetector(unsigned char preamble_length_2bits, S2LP_PreamblePattern preamble_pattern) {
	// Set length.
	unsigned char pcktctrlx_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_PCKTCTRL6, &pcktctrlx_reg_value);
	pcktctrlx_reg_value &= 0xFC;
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL6, pcktctrlx_reg_value);
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL5, preamble_length_2bits);
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

/* DISABLE CRC.
 * @param:	None.
 * @return:	None.
 */
void S2LP_DisableCrc(void) {
	// Read register.
	unsigned char reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_PCKTCTRL1, &reg_value);
	// Set bits.
	reg_value &= 0x1F;
	// Write register.
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL1, reg_value);
}

/* CONFIGURE TX POWER AMPLIFIER.
 * @param:	None.
 * @return:	None.
 */
void S2LP_ConfigurePa(void) {
	// Disable PA power ramping.
	unsigned char reg_value = 0;
	S2LP_WriteRegister(S2LP_REG_PA_POWER0, 0x07);
	// Disable FIR.
	S2LP_ReadRegister(S2LP_REG_PA_CONFIG1,  &reg_value);
	reg_value &= 0xFD;
	S2LP_WriteRegister(S2LP_REG_PA_CONFIG1, reg_value);
}

/* SET S2LP TX DATA SOURCE.
 * @param tx_source:	TX data source (use enumeration defined in s2lp.h).
 * @return:				None.
 */
void S2LP_SetTxSource(S2LP_TxSource tx_source) {
	// Read register.
	unsigned char reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_PCKTCTRL1, &reg_value);
	// Set bits.
	reg_value &= 0xF3;
	reg_value |= (tx_source << 2);
	// Write register.
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL1, reg_value);
}

/* INITIATE FIFO WRITING OPERATION.
 * @param:	None.
 * @return:	None.
 */
void S2LP_WriteFifo(unsigned char* tx_data, unsigned char tx_data_length_bytes) {
#ifdef S2LP_TX_FIFO_USE_DMA
	// Set buffer address.
	DMA1_SetChannel3SourceAddr((unsigned int) tx_data, tx_data_length_bytes);
#endif
	// Falling edge on CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 0);
	// Access FIFO.
	SPI1_WriteByte(S2LP_HEADER_BYTE_WRITE); // A/C='1' and W/R='0'.
	SPI1_WriteByte(S2LP_REG_FIFO);
#ifdef S2LP_TX_FIFO_USE_DMA
	// Transfer buffer with DMA.
	DMA1_StartChannel3();
	while (DMA1_GetChannel3Status() == 0) {
		PWR_EnterSleepMode();
	}
	DMA1_StopChannel3();
#else
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<tx_data_length_bytes ; byte_idx++) {
		SPI1_WriteByte(tx_data[byte_idx]);
	}
#endif
	// Rising edge on CS pin.
	GPIO_Write(&GPIO_S2LP_CS, 1);
}

/* SET S2LP RX SOURCE.
 * @param rx_source:	RX data source (use enumeration defined in s2lp.h).
 * @return:				None.
 */
void S2LP_SetRxSource(S2LP_RxSource rx_source) {
	// Read register.
	unsigned char reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_PCKTCTRL3, &reg_value);
	// Set bits.
	reg_value &= 0xCF;
	reg_value |= (rx_source << 4);
	// Write register.
	S2LP_WriteRegister(S2LP_REG_PCKTCTRL3, reg_value);
}

/* SET RX FILTER BANDWIDTH.
 * @param bit_rate_setting:	RX bandwidth mantissa and exponent setting.
 * @return:					None.
 */
void S2LP_SetRxBandwidth(S2LP_MantissaExponent rxbw_setting) {
	// Write register.
	unsigned char chflt_reg_value = ((rxbw_setting.mantissa << 4) & 0xF0) + (rxbw_setting.exponent & 0x0F);
	S2LP_WriteRegister(S2LP_REG_CHFLT, chflt_reg_value);
}

/* DISABLE CS, EQUALIZATION AND ANTENNA SWITCHING.
 * @param:	None.
 * @return:	None.
 */
void S2LP_DisableEquaCsAntSwitch(void) {
	// Read register.
	unsigned char ant_select_conf_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_ANT_SELECT_CONF, &ant_select_conf_reg_value);
	// Disable equalization.
	ant_select_conf_reg_value &= 0x83;
	// Program register.
	S2LP_WriteRegister(S2LP_REG_ANT_SELECT_CONF, ant_select_conf_reg_value);
}

/* GET CURRENT RSSI LEVEL.
 * @param:		None.
 * return rssi:	RSSI level captured at the end of the sync word detection (in dBm).
 */
signed int S2LP_GetRssi(void) {
	unsigned char rssi_level_reg_value = 0;
	S2LP_ReadRegister(S2LP_REG_RSSI_LEVEL, &rssi_level_reg_value);
	signed int rssi = rssi_level_reg_value - S2LP_RSSI_OFFSET_DB;
	return rssi;
}

/* READ S2LP RX FIFO.
 * @param rx_data:				Byte array that will contain FIFO data.
 * @param rx_data_length_bytes:	Number of bytes to read.
 */
void S2LP_ReadFifo(unsigned char* rx_data, unsigned char rx_data_length_bytes) {
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
