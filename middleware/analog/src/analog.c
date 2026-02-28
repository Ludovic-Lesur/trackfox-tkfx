/*
 * analog.c
 *
 *  Created on: 12 aug. 2024
 *      Author: Ludo
 */

#include "analog.h"

#include "adc.h"
#include "error.h"
#include "error_base.h"
#include "mcu_mapping.h"
#include "tkfx_flags.h"
#include "types.h"

/*** ANALOG local macros ***/

#define ANALOG_MCU_VOLTAGE_DEFAULT_MV           3000
#define ANALOG_MCU_TEMPERATURE_DEFAULT_DEGREES  25

#define ANALOG_LM4040_VOLTAGE_MV                2048

#define ANALOG_DIVIDER_RATIO_SOURCE_VOLTAGE     10
#ifdef TKFX_MODE_SUPERCAPACITOR
#define ANALOG_DIVIDER_RATIO_STORAGE_VOLTAGE    1
#endif
#ifdef TKFX_MODE_BATTERY
#define ANALOG_DIVIDER_RATIO_STORAGE_VOLTAGE    2
#endif

#define ANALOG_ERROR_VALUE                      0xFFFF

/*** ANALOG local structures ***/

/*******************************************************************/
typedef struct {
    int32_t mcu_voltage_mv;
    int32_t lm4040_voltage_12bits;
} ANALOG_context_t;

/*** ANALOG local global variables ***/

static ANALOG_context_t analog_ctx = {
    .mcu_voltage_mv = ANALOG_MCU_VOLTAGE_DEFAULT_MV,
    .lm4040_voltage_12bits = ANALOG_ERROR_VALUE
};

/*** ANALOG local functions ***/

/*******************************************************************/
static ANALOG_status_t _ANALOG_calibrate(void) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    int32_t adc_data_12bits = 0;
    // Convert external voltage reference.
    adc_status = ADC_convert_channel(ADC_CHANNEL_LM4040, &adc_data_12bits);
    ADC_exit_error(ANALOG_ERROR_BASE_ADC);
    // Update local calibration value.
    analog_ctx.lm4040_voltage_12bits = adc_data_12bits;
errors:
    return status;
}

/*** ANALOG functions ***/

/*******************************************************************/
ANALOG_status_t ANALOG_init(void) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    // Init context.
    analog_ctx.mcu_voltage_mv = ANALOG_MCU_VOLTAGE_DEFAULT_MV;
    analog_ctx.lm4040_voltage_12bits = ANALOG_ERROR_VALUE;
    // Init internal ADC.
    adc_status = ADC_init(&ADC_GPIO);
    ADC_exit_error(ANALOG_ERROR_BASE_ADC);
    // Calibration with external reference.
    status = _ANALOG_calibrate();
    if (status != ANALOG_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
ANALOG_status_t ANALOG_de_init(void) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    // Erase calibration value.
    analog_ctx.lm4040_voltage_12bits = ANALOG_ERROR_VALUE;
    // Release internal ADC.
    adc_status = ADC_de_init();
    ADC_stack_error(ERROR_BASE_ANALOG + ANALOG_ERROR_BASE_ADC);
    return status;
}

/*******************************************************************/
ANALOG_status_t ANALOG_convert_channel(ANALOG_channel_t channel, int32_t* analog_data) {
    // Local variables.
    ANALOG_status_t status = ANALOG_SUCCESS;
    ADC_status_t adc_status = ADC_SUCCESS;
    int32_t adc_data_12bits = 0;
    // Check parameter.
    if (analog_data == NULL) {
        status = ANALOG_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check channel.
    switch (channel) {
    case ANALOG_CHANNEL_MCU_VOLTAGE_MV:
        // MCU voltage.
        adc_status = ADC_convert_channel(ADC_CHANNEL_VREFINT, &adc_data_12bits);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to mV.
        adc_status = ADC_compute_mcu_voltage(adc_data_12bits, ADC_get_vrefint_voltage_mv(), analog_data);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Update local value for temperature computation.
        analog_ctx.mcu_voltage_mv = (*analog_data);
        break;
    case ANALOG_CHANNEL_MCU_TEMPERATURE_DEGREES:
        // MCU temperature.
        adc_status = ADC_convert_channel(ADC_CHANNEL_TEMPERATURE_SENSOR, &adc_data_12bits);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to degrees.
        adc_status = ADC_compute_mcu_temperature(analog_ctx.mcu_voltage_mv, adc_data_12bits, analog_data);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        break;
    case ANALOG_CHANNEL_SOURCE_VOLTAGE_MV:
        // Check calibration.
        if (analog_ctx.lm4040_voltage_12bits == ANALOG_ERROR_VALUE) {
            status = ANALOG_ERROR_CALIBRATION_MISSING;
            goto errors;
        }
        // Solar cell or dynamo voltage.
        adc_status = ADC_convert_channel(ADC_CHANNEL_SOURCE_VOLTAGE, &adc_data_12bits);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to mV.
        (*analog_data) = (adc_data_12bits * ANALOG_LM4040_VOLTAGE_MV * ANALOG_DIVIDER_RATIO_SOURCE_VOLTAGE) / (analog_ctx.lm4040_voltage_12bits);
        break;
    case ANALOG_CHANNEL_STORAGE_VOLTAGE_MV:
        // Check calibration.
        if (analog_ctx.lm4040_voltage_12bits == ANALOG_ERROR_VALUE) {
            status = ANALOG_ERROR_CALIBRATION_MISSING;
            goto errors;
        }
        // Supercap or battery voltage.
        adc_status = ADC_convert_channel(ADC_CHANNEL_STORAGE_VOLTAGE, &adc_data_12bits);
        ADC_exit_error(ANALOG_ERROR_BASE_ADC);
        // Convert to mV.
        (*analog_data) = (adc_data_12bits * ANALOG_LM4040_VOLTAGE_MV * ANALOG_DIVIDER_RATIO_STORAGE_VOLTAGE) / (analog_ctx.lm4040_voltage_12bits);
        break;
    default:
        status = ANALOG_ERROR_CHANNEL;
        goto errors;
    }
errors:
    return status;
}
