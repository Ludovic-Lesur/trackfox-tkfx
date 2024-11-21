/*
 * power.c
 *
 *  Created on: 22 jul. 2023
 *      Author: Ludo
 */

#include "power.h"

#include "analog.h"
#include "error.h"
#include "gpio.h"
#include "gps.h"
#include "lptim.h"
#include "gpio_mapping.h"
#include "s2lp.h"
#include "sht3x.h"
#include "types.h"

/*** POWER local global variables ***/

static uint8_t power_domain_state[POWER_DOMAIN_LAST];

/*** POWER functions ***/

/*******************************************************************/
void POWER_init(void) {
    // Local variables.
    POWER_domain_t domain = 0;
    // Init power control pins.
    GPIO_configure(&GPIO_ADC_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_GPS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_SENSORS_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Disable all domains by default.
    for (domain = 0; domain < POWER_DOMAIN_LAST; domain++) {
        POWER_disable(domain);
    }
}

/*******************************************************************/
POWER_status_t POWER_enable(POWER_domain_t domain, LPTIM_delay_mode_t delay_mode) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    S2LP_status_t s2lp_status = S2LP_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    MMA865XFC_status_t mma865xfc_status = MMA865XFC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    uint32_t delay_ms = 0;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
        // Turn analog front-end on.
        GPIO_write(&GPIO_ADC_POWER_ENABLE, 1);
        // Init ADC.
        analog_status = ANALOG_init();
        ANALOG_exit_error(POWER_ERROR_BASE_ANALOG);
        delay_ms = POWER_ON_DELAY_MS_ANALOG;
        break;
    case POWER_DOMAIN_SENSORS:
        // Turn digital sensors on and init common I2C interface.
        GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
        // Init sensors drivers.
        sht3x_status = SHT3X_init();
        SHT3X_exit_error(POWER_ERROR_BASE_SHT30);
        mma865xfc_status = MMA865XFC_init();
        MMA865XFC_exit_error(POWER_ERROR_BASE_MMA8653FC);
        delay_ms = POWER_ON_DELAY_MS_SENSORS;
        break;
    case POWER_DOMAIN_GPS:
        // Turn GPS on.
        GPIO_write(&GPIO_GPS_POWER_ENABLE, 1);
        // Init GPS driver.
        gps_status = GPS_init();
        GPS_exit_error(POWER_ERROR_BASE_GPS);
        delay_ms = POWER_ON_DELAY_MS_GPS;
        break;
    case POWER_DOMAIN_TCXO:
        // Turn TCXO on.
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_TCXO;
        break;
    case POWER_DOMAIN_RADIO:
        // Turn radio on.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
        // Init transceiver.
        s2lp_status = S2LP_init();
        S2LP_exit_error(POWER_ERROR_BASE_S2LP);
        delay_ms = POWER_ON_DELAY_MS_RADIO;
        break;
    default:
        status = POWER_ERROR_DOMAIN;
        goto errors;
    }
    // Update state.
    power_domain_state[domain] = 1;
    // Power on delay.
    if (delay_ms != 0) {
        lptim_status = LPTIM_delay_milliseconds(delay_ms, delay_mode);
        LPTIM_exit_error(POWER_ERROR_BASE_LPTIM);
    }
errors:
    return status;
}

/*******************************************************************/
POWER_status_t POWER_disable(POWER_domain_t domain) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    S2LP_status_t s2lp_status = S2LP_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    MMA865XFC_status_t mma865xfc_status = MMA865XFC_SUCCESS;
    // Check domain.
    switch (domain) {
    case POWER_DOMAIN_ANALOG:
        // Release ADC.
        analog_status = ANALOG_de_init();
        ANALOG_exit_error(POWER_ERROR_BASE_ANALOG);
        // Turn analog front-end off.
        GPIO_write(&GPIO_ADC_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_SENSORS:
        // Release sensors drivers.
        sht3x_status = SHT3X_de_init();
        SHT3X_exit_error(POWER_ERROR_BASE_SHT30);
        mma865xfc_status = MMA865XFC_de_init();
        MMA865XFC_exit_error(POWER_ERROR_BASE_MMA8653FC);
        // Turn digital sensors off and release I2C interface.
        GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_GPS:
        // Release GPS driver.
        gps_status = GPS_de_init();
        GPS_exit_error(POWER_ERROR_BASE_GPS);
        // Turn GPS off.
        GPIO_write(&GPIO_GPS_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_TCXO:
        // Turn TCXO off.
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_RADIO:
        // Release transceiver.
        s2lp_status = S2LP_de_init();
        S2LP_exit_error(POWER_ERROR_BASE_S2LP);
        // Turn radio off and release S2LP driver.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
        break;
    default:
        status = POWER_ERROR_DOMAIN;
        goto errors;
    }
    // Update state.
    power_domain_state[domain] = 0;
errors:
    return status;
}

/*******************************************************************/
POWER_status_t POWER_get_state(POWER_domain_t domain, uint8_t* state) {
    // Local variables.
    POWER_status_t status = POWER_SUCCESS;
    // Check parameters.
    if (domain >= POWER_DOMAIN_LAST) {
        status = POWER_ERROR_DOMAIN;
        goto errors;
    }
    if (state == NULL) {
        status = POWER_ERROR_NULL_PARAMETER;
        goto errors;
    }
    (*state) = power_domain_state[domain];
errors:
    return status;
}
