/*
 * adc.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "adc.h"

#include "adc_reg.h"
#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "rcc_reg.h"

/*** ADC local macros ***/

#define ADC_TIMEOUT_COUNT					1000000
#define ADC_CHANNEL_SOURCE					6
#define ADC_CHANNEL_SUPERCAP				7
#define ADC_CHANNEL_LM4040					8
#define ADC_LM4040_VOLTAGE_MV				2048
#define ADC_FULL_SCALE_12BITS				4095
#define ADC_SOURCE_VOLTAGE_DIVIDER_R_UP		470
#define ADC_SOURCE_VOLTAGE_DIVIDER_R_DOWN	68

/*** ADC local structures ***/

typedef struct {
	unsigned int adc_lm4040_voltage_12bits;
	unsigned int adc_source_voltage_mv;
	unsigned int adc_supercap_voltage_mv;
	unsigned int adc_mcu_voltage_mv;
} ADC_Context;

/*** ADC local global variables ***/

static ADC_Context adc_ctx;

/*** ADC local functions ***/

/* UPDATE EXTERNAL BANDGAP CONVERSION RESULT.
 * @param:	None.
 * @return:	None.
 */
void ADC1_UpdateLm4040Result(void) {
	// Select input channel.
	ADC1 -> CHSELR &= 0xFFF80000; // Reset all bits.
	ADC1 -> CHSELR |= (0b1 << ADC_CHANNEL_LM4040);
	// Read raw supply voltage.
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	unsigned int loop_count = 0;
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0) {
		// Wait end of conversion ('EOC='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) return;
	}
	adc_ctx.adc_lm4040_voltage_12bits = (ADC1 -> DR);
}

/* COMPUTE SOURCE VOLTAGE.
 * @param:	None.
 * @return:	None.
 */
void ADC1_ComputeSourceVoltage(void) {
	// Select input channel.
	ADC1 -> CHSELR &= 0xFFF80000; // Reset all bits.
	ADC1 -> CHSELR |= (0b1 << ADC_CHANNEL_SOURCE);
	// Read raw supply voltage.
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	unsigned int loop_count = 0;
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0) {
		// Wait end of conversion ('EOC='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) return;
	}
	unsigned int source_voltage_12bits = (ADC1 -> DR);
	// Convert to mV using bandgap result.
	adc_ctx.adc_source_voltage_mv = (ADC_LM4040_VOLTAGE_MV * source_voltage_12bits) / (adc_ctx.adc_lm4040_voltage_12bits);
	// Compensate voltage divider ratio.
	adc_ctx.adc_source_voltage_mv = (adc_ctx.adc_source_voltage_mv * (ADC_SOURCE_VOLTAGE_DIVIDER_R_UP + ADC_SOURCE_VOLTAGE_DIVIDER_R_DOWN)) / (ADC_SOURCE_VOLTAGE_DIVIDER_R_DOWN);
}

/* COMPUTE SUPERCAP VOLTAGE.
 * @param:	None.
 * @return:	None.
 */
void ADC1_ComputeSupercapVoltage(void) {
	// Select input channel.
	ADC1 -> CHSELR &= 0xFFF80000; // Reset all bits.
	ADC1 -> CHSELR |= (0b1 << ADC_CHANNEL_SUPERCAP);
	// Read raw supply voltage.
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	unsigned int loop_count = 0;
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0) {
		// Wait end of conversion ('EOC='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) return;
	}
	unsigned int supercap_voltage_12bits = (ADC1 -> DR);
	// Convert to mV using bandgap result.
	adc_ctx.adc_supercap_voltage_mv = (ADC_LM4040_VOLTAGE_MV * supercap_voltage_12bits) / (adc_ctx.adc_lm4040_voltage_12bits);
}

/* COMPUTE MCU SUPPLY VOLTAGE.
 * @param:	None.
 * @return:	None.
 */
void ADC1_ComputeMcuVoltage(void) {
	// Retrieve supply voltage from bandgap result.
	adc_ctx.adc_mcu_voltage_mv = (ADC_LM4040_VOLTAGE_MV * ADC_FULL_SCALE_12BITS) / (adc_ctx.adc_lm4040_voltage_12bits);
}

/*** ADC functions ***/

/* INIT ADC1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void ADC1_Init(void) {
	// Init GPIOs.
	GPIO_Configure(&GPIO_ADC_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_ADC1_IN6, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_ADC1_IN7, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_ADC1_IN8, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Init context.
	adc_ctx.adc_lm4040_voltage_12bits = 0;
	adc_ctx.adc_source_voltage_mv = 0;
	adc_ctx.adc_supercap_voltage_mv = 0;
	adc_ctx.adc_mcu_voltage_mv = 0;
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 9); // ADCEN='1'.
	// Ensure ADC is disabled.
	if (((ADC1 -> CR) & (0b1 << 0)) != 0) {
		ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	}
	// Enable ADC voltage regulator.
	ADC1 -> CR |= (0b1 << 28);
	LPTIM1_DelayMilliseconds(5);
	// ADC configuration.
	ADC1 -> CFGR2 &= ~(0b11 << 30); // Reset bits 30-31.
	ADC1 -> CFGR2 |= (0b01 << 30); // Use (PCLK2/2) as ADCCLK = SYSCLK/2 (see RCC_Init() function).
	ADC1 -> CFGR1 &= (0b1 << 13); // Single conversion mode.
	ADC1 -> CFGR1 &= ~(0b11 << 0); // Data resolution = 12 bits (RES='00').
	ADC1 -> CCR &= 0xFC03FFFF; // No prescaler.
	ADC1 -> SMPR |= (0b111 << 0); // Maximum sampling time.
	// ADC calibration.
	ADC1 -> CR |= (0b1 << 31); // ADCAL='1'.
	unsigned int loop_count = 0;
	while ((((ADC1 -> CR) & (0b1 << 31)) != 0) && (((ADC1 -> ISR) & (0b1 << 11)) == 0)) {
		// Wait until calibration is done or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) break;
	}
	// Clear all flags.
	ADC1 -> ISR |= 0x0000089F;
}

/* DISABLE INTERNAL ADC PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void ADC1_Disable(void) {
	// Disable power control pin (other inputs are allready in analog mode).
	GPIO_Configure(&GPIO_ADC_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable peripheral.
	if (((ADC1 -> CR) & (0b1 << 0)) != 0) {
		ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	}
	// Clear all flags.
	ADC1 -> ISR |= 0x0000089F;
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 9); // ADCEN='0'.
}

/* ENABLE EXTERNAL ANALOG BLOCKS SUPPLY.
 * @param:	None.
 * @return:	None.
 */
void ADC1_PowerOn(void) {
	// Turn analog blocks on.
	GPIO_Write(&GPIO_ADC_POWER_ENABLE, 1);
	LPTIM1_DelayMilliseconds(100);
}

/* DISABLE EXTERNAL ANALOG BLOCKS SUPPLY.
 * @param:	None.
 * @return:	None.
 */
void ADC1_PowerOff(void) {
	// Turn analog blocks on.
	GPIO_Write(&GPIO_ADC_POWER_ENABLE, 0);
}

/* PERFORM INTERNAL ADC MEASUREMENTS.
 * @param:	None.
 * @return:	None.
 */
void ADC1_PerformMeasurements(void) {
	// Enable ADC peripheral.
	ADC1 -> CR |= (0b1 << 0); // ADEN='1'.
	unsigned int loop_count = 0;
	while (((ADC1 -> ISR) & (0b1 << 0)) == 0) {
		// Wait for ADC to be ready (ADRDY='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) return;
	}
	// Perform measurements.
	ADC1_UpdateLm4040Result();
	ADC1_ComputeSourceVoltage();
	ADC1_ComputeSupercapVoltage();
	ADC1_ComputeMcuVoltage();
	// Clear all flags.
	ADC1 -> ISR |= 0x0000089F; // Clear all flags.
	// Disable ADC peripheral.
	if (((ADC1 -> CR) & (0b1 << 0)) != 0) {
		ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	}
}

/* GET SOURCE VOLTAGE.
 * @param source_voltage_mv:	Pointer to value that will contain source voltage in mV.
 * @return:						None.
 */
void ADC1_GetSourceVoltage(unsigned int* source_voltage_mv) {
	(*source_voltage_mv) = adc_ctx.adc_source_voltage_mv;
}

/* GET SUPERCAO VOLTAGE.
 * @param supercap_voltage_mv:	Pointer to value that will contain supercap voltage in mV.
 * @return:						None.
 */
void ADC1_GetSupercapVoltage(unsigned int* supercap_voltage_mv) {
	(*supercap_voltage_mv) = adc_ctx.adc_supercap_voltage_mv;
}

/* GET MCU SUPPLY VOLTAGE.
 * @param supply_voltage_mv:	Pointer to value that will contain MCU supply voltage in mV.
 * @return:						None.
 */
void ADC1_GetMcuVoltage(unsigned int* mcu_voltage_mv) {
	(*mcu_voltage_mv) = adc_ctx.adc_mcu_voltage_mv;
}
