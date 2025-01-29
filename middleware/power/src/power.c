/*
 * power.c
 *
 *  Created on: 22 jul. 2023
 *      Author: Ludo
 */

#include "power.h"

#include "analog.h"
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "gpio_mapping.h"
#include "gps.h"
#include "lptim.h"
#include "s2lp.h"
#include "sht3x.h"
#include "types.h"

/*** POWER local global variables ***/

static uint32_t power_domain_state[POWER_DOMAIN_LAST];

/*** POWER local functions ***/

/*******************************************************************/
#define _POWER_stack_driver_error(driver_status, driver_success, driver_error_base, power_status) { \
    if (driver_status != driver_success) { \
        ERROR_stack_add(driver_error_base + driver_status); \
        ERROR_stack_add(ERROR_BASE_POWER + power_status); \
    } \
}

/*** POWER functions ***/

/*******************************************************************/
void POWER_init(void) {
    // Local variables.
    uint8_t idx = 0;
    // Init context.
    for (idx = 0; idx < POWER_DOMAIN_LAST; idx++) {
        power_domain_state[idx] = 0;
    }
    // Init power control pins.
    GPIO_configure(&GPIO_ADC_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_GPS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_SENSORS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/*******************************************************************/
void POWER_enable(POWER_requester_id_t requester_id, POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    S2LP_status_t s2lp_status = S2LP_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    MMA865XFC_status_t mma865xfc_status = MMA865XFC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    uint32_t delay_ms = 0;
    uint8_t action_required = 0;
    // Check parameters.
    if (requester_id >= POWER_REQUESTER_ID_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_REQUESTER_ID);
        goto errors;
    }
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    action_required = ((power_domain_state[domain] == 0) ? 1 : 0);
    // Update state.
    power_domain_state[domain] |= (0b1 << requester_id);
    // Directly exit if this is not the first request.
    if (action_required == 0) goto errors;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
        // Turn analog front-end on.
        GPIO_write(&GPIO_ADC_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_ANALOG;
        // Init attached drivers.
        analog_status = ANALOG_init();
        _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
        break;
    case POWER_DOMAIN_SENSORS:
        // Turn digital sensors.
        GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_SENSORS;
        // Init attached drivers.
        sht3x_status = SHT3X_init();
        _POWER_stack_driver_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30, POWER_ERROR_DRIVER_SHT3X);
        mma865xfc_status = MMA865XFC_init();
        _POWER_stack_driver_error(mma865xfc_status, MMA865XFC_SUCCESS, ERROR_BASE_MMA8653FC, POWER_ERROR_DRIVER_MMA865XFC);
        break;
    case POWER_DOMAIN_GPS:
        // Turn GPS on.
        GPIO_write(&GPIO_GPS_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_GPS;
        // Init attached drivers.
        gps_status = GPS_init();
        _POWER_stack_driver_error(gps_status, GPS_SUCCESS, ERROR_BASE_GPS, POWER_ERROR_DRIVER_GPS);
        break;
    case POWER_DOMAIN_TCXO:
        // Turn TCXO on.
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_TCXO;
        break;
    case POWER_DOMAIN_RADIO:
        // Turn radio on.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_RADIO;
        // Init attached drivers.
        s2lp_status = S2LP_init();
        _POWER_stack_driver_error(s2lp_status, S2LP_SUCCESS, ERROR_BASE_S2LP, POWER_ERROR_DRIVER_S2LP);
        break;
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    // Power on delay.
    if (delay_ms != 0) {
        lptim_status = LPTIM_delay_milliseconds(delay_ms, delay_mode);
        _POWER_stack_driver_error(lptim_status, LPTIM_SUCCESS, ERROR_BASE_LPTIM, POWER_ERROR_DRIVER_LPTIM);
    }
errors:
    return;
}

/*******************************************************************/
void POWER_disable(POWER_requester_id_t requester_id, POWER_domain_t domain) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    S2LP_status_t s2lp_status = S2LP_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    MMA865XFC_status_t mma865xfc_status = MMA865XFC_SUCCESS;
    // Check parameters.
    if (requester_id >= POWER_REQUESTER_ID_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_REQUESTER_ID);
        goto errors;
    }
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    if (power_domain_state[domain] == 0) goto errors;
    // Update state.
    power_domain_state[domain] &= ~(0b1 << requester_id);
    // Directly exit if this is not the last request.
    if (power_domain_state[domain] != 0) goto errors;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
        // Release attached drivers.
        analog_status = ANALOG_de_init();
        _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
        // Turn analog front-end off.
        GPIO_write(&GPIO_ADC_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_SENSORS:
        // Release attached drivers.
        sht3x_status = SHT3X_de_init();
        _POWER_stack_driver_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30, POWER_ERROR_DRIVER_SHT3X);
        mma865xfc_status = MMA865XFC_de_init();
        _POWER_stack_driver_error(mma865xfc_status, MMA865XFC_SUCCESS, ERROR_BASE_MMA8653FC, POWER_ERROR_DRIVER_MMA865XFC);
        // Turn digital sensors off.
        GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_GPS:
        // Release attached drivers.
        gps_status = GPS_de_init();
        _POWER_stack_driver_error(gps_status, GPS_SUCCESS, ERROR_BASE_GPS, POWER_ERROR_DRIVER_GPS);
        // Turn GPS off.
        GPIO_write(&GPIO_GPS_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_TCXO:
        // Turn TCXO off.
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_RADIO:
        // Release attached drivers.
        s2lp_status = S2LP_de_init();
        _POWER_stack_driver_error(s2lp_status, S2LP_SUCCESS, ERROR_BASE_S2LP, POWER_ERROR_DRIVER_S2LP);
        // Turn radio off.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
        break;
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
errors:
    return;
}

/*******************************************************************/
uint8_t POWER_get_state(POWER_domain_t domain) {
    // Local variables.
    uint8_t state = 0;
    // Check parameters.
    if (domain >= POWER_DOMAIN_LAST) {
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
    state = (power_domain_state[domain] == 0) ? 0 : 1;
errors:
    return state;
}
