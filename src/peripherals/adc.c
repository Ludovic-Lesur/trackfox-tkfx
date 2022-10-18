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
#include "math.h"
#include "rcc_reg.h"
#include "types.h"

/*** ADC local macros ***/

#define ADC_CHANNEL_VSRC				6
#define ADC_CHANNEL_VCAP				7
#define ADC_CHANNEL_LM4040				8
#define ADC_CHANNEL_TMCU				18

#define ADC_MEDIAN_FILTER_LENGTH		9
#define ADC_CENTER_AVERAGE_LENGTH		3

#define ADC_FULL_SCALE_12BITS			4095

#define ADC_LM4040_VOLTAGE_MV			2048
#define ADC_VMCU_DEFAULT_MV				3000

#define ADC_VOLTAGE_DIVIDER_RATIO_VSRC	10

#define ADC_TIMEOUT_COUNT				1000000

/*** ADC local structures ***/

typedef struct {
	uint32_t lm4040_12bits;
	uint32_t data[ADC_DATA_INDEX_LAST];
	int8_t tmcu_degrees;
} ADC_context_t;

/*** ADC local global variables ***/

static ADC_context_t adc_ctx;

/*** ADC local functions ***/

/* PERFORM A SINGLE ADC CONVERSION.
 * @param adc_channel:			Channel to convert.
 * @param adc_result_12bits:	Pointer to int that will contain ADC raw result on 12 bits.
 * @return status:				Function execution status.
 */
static ADC_status_t _ADC1_single_conversion(uint8_t adc_channel, uint32_t* adc_result_12bits) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t loop_count = 0;
	// Select input channel.
	ADC1 -> CHSELR &= 0xFFF80000; // Reset all bits.
	ADC1 -> CHSELR |= (0b1 << adc_channel);
	// Clear all flags.
	ADC1 -> ISR |= 0x0000089F;
	// Read raw supply voltage.
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0) {
		// Wait end of conversion ('EOC='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_TIMEOUT;
			goto errors;
		}
	}
	(*adc_result_12bits) = (ADC1 -> DR);
errors:
	return status;
}

/* PERFORM SEVERAL CONVERSIONS FOLLOWED BY A MEDIAN FILTER.
 * @param adc_channel:			Channel to convert.
 * @param adc_result_12bits:	Pointer to int that will contain ADC filtered result on 12 bits.
 * @return status:				Function execution status.
 */
static ADC_status_t _ADC1_filtered_conversion(uint8_t adc_channel, uint32_t* adc_result_12bits) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	uint32_t adc_sample_buf[ADC_MEDIAN_FILTER_LENGTH] = {0x00};
	uint8_t idx = 0;
	// Perform all conversions.
	for (idx=0 ; idx<ADC_MEDIAN_FILTER_LENGTH ; idx++) {
		status = _ADC1_single_conversion(adc_channel, &(adc_sample_buf[idx]));
		if (status != ADC_SUCCESS) goto errors;
	}
	// Apply median filter.
	math_status = MATH_median_filter_u32(adc_sample_buf, ADC_MEDIAN_FILTER_LENGTH, ADC_CENTER_AVERAGE_LENGTH, adc_result_12bits);
errors:
	return status;
}

/* PERFORM EXTERNAL REFERENCE VOLTAGE CONVERSION.
 * @param:			None.
 * @return status:	Function execution status.
 */
static ADC_status_t ADC1_compute_lm4040(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	// Read raw reference voltage.
	status = _ADC1_filtered_conversion(ADC_CHANNEL_LM4040, &adc_ctx.lm4040_12bits);
	return status;
}

/* COMPUTE MCU SUPPLY VOLTAGE.
 * @param:			None.
 * @return status:	Function execution status.
 */
static void _ADC1_compute_vmcu(void) {
	// Retrieve supply voltage from bandgap result.
	adc_ctx.data[ADC_DATA_INDEX_VMCU_MV] = (ADC_LM4040_VOLTAGE_MV * ADC_FULL_SCALE_12BITS) / (adc_ctx.lm4040_12bits);
}

/* COMPUTE MCU TEMPERATURE THANKS TO INTERNAL VOLTAGE REFERENCE.
 * @param:			None.
 * @return status:	Function execution status.
 */
static ADC_status_t _ADC1_compute_tmcu(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t raw_temp_sensor_12bits = 0;
	int raw_temp_calib_mv = 0;
	int temp_calib_degrees = 0;
	// Read raw temperature.
	status = _ADC1_filtered_conversion(ADC_CHANNEL_TMCU, &raw_temp_sensor_12bits);
	if (status != ADC_SUCCESS) goto errors;
	// Compute temperature according to MCU factory calibration (see p.301 and p.847 of RM0377 datasheet).
	raw_temp_calib_mv = ((int) raw_temp_sensor_12bits * adc_ctx.data[ADC_DATA_INDEX_VMCU_MV]) / (TS_VCC_CALIB_MV) - TS_CAL1; // Equivalent raw measure for calibration power supply (VCC_CALIB).
	temp_calib_degrees = raw_temp_calib_mv * ((int) (TS_CAL2_TEMP-TS_CAL1_TEMP));
	temp_calib_degrees = (temp_calib_degrees) / ((int) (TS_CAL2 - TS_CAL1));
	adc_ctx.tmcu_degrees = temp_calib_degrees + TS_CAL1_TEMP;
errors:
	return status;
}

/* COMPUTE SOURCE VOLTAGE.
 * @param:			None.
 * @return status:	Function execution status.
 */
static ADC_status_t ADC1_compute_vsrc(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t vsrc_12bits = 0;
	// Get raw result.
	status = _ADC1_filtered_conversion(ADC_CHANNEL_VSRC, &vsrc_12bits);
	if (status != ADC_SUCCESS) goto errors;
	// Convert to mV using bandgap result.
	adc_ctx.data[ADC_DATA_INDEX_VSRC_MV] = (ADC_LM4040_VOLTAGE_MV * vsrc_12bits * ADC_VOLTAGE_DIVIDER_RATIO_VSRC) / (adc_ctx.lm4040_12bits);
errors:
	return status;
}

/* COMPUTE SUPERCAP VOLTAGE.
 * @param:			None.
 * @return status:	Function execution status.
 */
static ADC_status_t ADC1_compute_vcap(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t vcap_12bits = 0;
	// Get raw result.
	status = _ADC1_filtered_conversion(ADC_CHANNEL_VCAP, &vcap_12bits);
	if (status != ADC_SUCCESS) goto errors;
	// Convert to mV using bandgap result.
	adc_ctx.data[ADC_DATA_INDEX_VCAP_MV] = (ADC_LM4040_VOLTAGE_MV * vcap_12bits) / (adc_ctx.lm4040_12bits);
errors:
	return status;
}

/*** ADC functions ***/

/* INIT ADC1 PERIPHERAL.
 * @param:			None.
 * @return status:	Function execution status.
 */
ADC_status_t ADC1_init(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint8_t idx = 0;
	uint32_t loop_count = 0;
	// Init context.
	adc_ctx.lm4040_12bits = 0;
	for (idx=0 ; idx<ADC_DATA_INDEX_LAST ; idx++) adc_ctx.data[idx] = 0;
	adc_ctx.data[ADC_DATA_INDEX_VMCU_MV] = ADC_VMCU_DEFAULT_MV;
	adc_ctx.tmcu_degrees = 0;
	// Init GPIOs.
	GPIO_configure(&GPIO_ADC_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_ADC1_IN6, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_ADC1_IN7, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure(&GPIO_ADC1_IN8, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 9); // ADCEN='1'.
	// Ensure ADC is disabled.
	if (((ADC1 -> CR) & (0b1 << 0)) != 0) {
		ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	}
	// Enable ADC voltage regulator.
	ADC1 -> CR |= (0b1 << 28);
	lptim1_status = LPTIM1_delay_milliseconds(5, 0);
	LPTIM1_status_check(ADC_ERROR_BASE_LPTIM);
	// ADC configuration.
	ADC1 -> CFGR2 |= (0b01 << 30); // Use (PCLK2/2) as ADCCLK = SYSCLK/2 (see RCC_init() function).
	ADC1 -> SMPR |= (0b111 << 0); // Maximum sampling time.
	// ADC calibration.
	ADC1 -> CR |= (0b1 << 31); // ADCAL='1'.
	while ((((ADC1 -> CR) & (0b1 << 31)) != 0) && (((ADC1 -> ISR) & (0b1 << 11)) == 0)) {
		// Wait until calibration is done or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_CALIBRATION;
			break;
		}
	}
errors:
	return status;
}

/* ENABLE EXTERNAL ANALOG BLOCKS SUPPLY.
 * @param:			None.
 * @return status:	Function execution status.
 */
ADC_status_t ADC1_power_on(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	// Turn analog blocks on.
	GPIO_write(&GPIO_ADC_POWER_ENABLE, 1);
	lptim1_status = LPTIM1_delay_milliseconds(100, 0);
	LPTIM1_status_check(ADC_ERROR_BASE_LPTIM);
errors:
	return status;
}

/* DISABLE EXTERNAL ANALOG BLOCKS SUPPLY.
 * @param:	None.
 * @return:	None.
 */
void ADC1_power_off(void) {
	// Turn analog blocks on.
	GPIO_write(&GPIO_ADC_POWER_ENABLE, 0);
}

/* PERFORM INTERNAL ADC MEASUREMENTS.
 * @param:			None.
 * @return status:	Function execution status.
 */
ADC_status_t ADC1_perform_measurements(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	uint32_t loop_count = 0;
	// Enable ADC peripheral.
	ADC1 -> CR |= (0b1 << 0); // ADEN='1'.
	while (((ADC1 -> ISR) & (0b1 << 0)) == 0) {
		// Wait for ADC to be ready (ADRDY='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_TIMEOUT;
			goto errors;
		}
	}
	// Wake-up VREFINT and temperature sensor.
	ADC1 -> CCR |= (0b11 << 22); // TSEN='1' and VREFEF='1'.
	// Wait internal reference stabilization (max 3ms).
	lptim1_status = LPTIM1_delay_milliseconds(10, 0);
	LPTIM1_status_check(ADC_ERROR_BASE_LPTIM);
	// Perform measurements.
	status = ADC1_compute_lm4040();
	if (status != ADC_SUCCESS) goto errors;
	_ADC1_compute_vmcu();
	status = _ADC1_compute_tmcu();
	if (status != ADC_SUCCESS) goto errors;
	status = ADC1_compute_vsrc();
	if (status != ADC_SUCCESS) goto errors;
	status = ADC1_compute_vcap();
errors:
	// Switch internal voltage reference off.
	ADC1 -> CCR &= ~(0b11 << 22); // TSEN='0' and VREFEF='0'.
	// Disable ADC peripheral.
	ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	return status;
}

/* GET ADC DATA.
 * @param data_idx:	Index of the data to retrieve.
 * @param data:		Pointer that will contain ADC data.
 * @return status:	Function execution status.
 */
ADC_status_t ADC1_get_data(ADC_data_index_t data_idx, uint32_t* data) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	// Check index.
	if (data_idx >= ADC_DATA_INDEX_LAST) {
		status = ADC_ERROR_DATA_INDEX;
		goto errors;
	}
	(*data) = adc_ctx.data[data_idx];
errors:
	return status;
}

/* GET MCU TEMPERATURE.
 * @param tmcu_degrees:	Pointer to signed value that will contain MCU temperature in degrees (2-complement).
 * @return:				None.
 */
void ADC1_get_tmcu(int8_t* tmcu_degrees) {
	(*tmcu_degrees) = adc_ctx.tmcu_degrees;
}
