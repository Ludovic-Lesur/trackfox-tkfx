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

#define S2LP_HEADER_BYTE_WRITE		0x00
#define S2LP_HEADER_BYTE_READ		0x01
#define S2LP_HEADER_BYTE_COMMAND	0x80

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

}

/* SET S2LP MODULATION SCHEME.
 * @param modulation:	Selected modulation (use enum defined in s2lp.h).
 * @return:				None.
 */
void S2LP_SetModulation(S2LP_Modulation modulation) {

}

/* SET TRANSCEIVER RF CENTRAL FREQUENCY.
 * @param rf_frequency_hz:	RF frequency in Hz.
 * @return:					None.
 */
void S2LP_SetRfFrequency(unsigned int rf_frequency_hz) {

}

/* SET FSK DEVIATION.
 * @param fsk_deviation_hz:	FSK deviation in Hz.
 * @return:					None.
 */
void S2LP_SetFskDeviation(unsigned short fsk_deviation_hz) {

}

/* SRT DATA BIT RATE.
 * @param bit_rate_bps:	Bit rate in bits/s.
 * @return:				None.
 */
void S2LP_SetBitRate(unsigned int bit_rate_bps) {

}

/* CONFIGURE S2LP GPIOs.
 * @param gpio_number:		GPIO to configure (0 to 3).
 * @param gpio_mode:		GPIO mode (use enum defined in s2lp.h).
 * @param gpio_function:	GPIO function (use enum defined in s2lp.h).
 * @return:					None.
 */
void S2LP_ConfigureGpio(unsigned char gpio_number, S2LP_GPIO_Mode gpio_mode, unsigned char gpio_function) {

}

/* GET S2LP IRQ FLAGS.
 * @param:	None.
 * @return:	TBD.
 */
unsigned short S2LP_GetIrqFlags(void) {
	return 0;
}

/* SET TX RF OUTPUT POER.
 * @param rf_output_power_dbm:	RF output power in dBm.
 * @return:						None.
 */
void S2LP_SetRfOutputPower(signed char rf_output_power_dbm) {

}

/* SET RX FILTER BANDWIDTH.
 * @param rxb_mantissa:		Channel filter bandwidth mantissa (see table p.35 of S2LP datasheet).
 * @param rxb_exponent:		Channel filter bandwidth exponent (see table p.35 of S2LP datasheet).
 * @return:					None.
 */
void S2LP_SetRxBandwidth(unsigned char rxbw_mantissa, unsigned char rxbw_exponent) {

}

/* SET RX PREAMBLE DETECTOR LENGTH.
 * @param preamble_length_bytes:	Preamble length in bytes.
 * @param preamble_polarity:		Preamble polarity (0/1).
 * @return:							None.
 */
void S2LP_SetPreambleDetector(unsigned char preamble_length_bytes, unsigned char preamble_polarity) {

}

/* CONFIGURE RX SYNC WORD DETECTOR.
 * @param sync_word:				Byte array containing the synchronization word.
 * @param sync_word_length_bytes:	Length of the synchronization word in bytes.
 * @return:							None.
 */
void S2LP_SetSyncWord(unsigned char* sync_word, unsigned char sync_word_length_bytes) {

}

/* CONFIGURE RX DATA LENGTH.
 * @param data_length_bytes:	RX data packet length in bytes.
 * @return:						None.
 */
void S2LP_SetDataLength(unsigned char data_length_bytes) {

}

/* GET CURRENT RSSI LEVEL.
 * @param:		None.
 * return rssi:	Current RSSI level in dBm.
 */
unsigned char S2LP_GetRssi(void) {
	return 0;
}

/* READ S2LP RX FIFO.
 * @param rx_data:				Byte array that will contain FIFO data.
 * @param rx_data_length_bytes:	Number of bytes to read.
 */
void S2LP_ReadRxFifo(unsigned char* rx_data, unsigned char rx_data_length_bytes) {

}
