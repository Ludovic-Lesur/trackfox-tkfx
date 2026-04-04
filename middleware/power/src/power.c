/*
 * power.c
 *
 *  Created on: 22 jul. 2023
 *      Author: Ludo
 */

#include "power.h"

#include "accelerometer.h"
#include "analog.h"
#include "error.h"
#include "error_base.h"
#include "fxls89xxxx.h"
#include "gpio.h"
#include "gps.h"
#include "lptim.h"
#include "mcu_mapping.h"
#include "rfe.h"
#include "s2lp.h"
#include "sht3x.h"
#include "types.h"
#include "wifi.h"

/*** POWER local global variables ***/

static uint32_t power_domain_state[POWER_DOMAIN_LAST] = { [0 ... (POWER_DOMAIN_LAST - 1)] = 0 };

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
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    ACCELEROMETER_status_t accelerometer_status = ACCELEROMETER_SUCCESS;
#ifdef HW2_0
    LED_status_t led_status = LED_SUCCESS;
    WIFI_status_t wifi_status = WIFI_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
#else
    S2LP_status_t s2lp_status = S2LP_SUCCESS;
#endif
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
        led_status = LED_set_color(LED_COLOR_GREEN);
        _POWER_stack_driver_error(led_status, LED_SUCCESS, ERROR_BASE_LED, POWER_ERROR_DRIVER_LED);
        analog_status = ANALOG_init();
        _POWER_stack_driver_error(analog_status, ANALOG_SUCCESS, ERROR_BASE_ANALOG, POWER_ERROR_DRIVER_ANALOG);
        break;
    case POWER_DOMAIN_SENSORS:
        // Turn digital sensors.
        GPIO_write(&GPIO_SENSORS_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_SENSORS;
        // Init attached drivers.
        led_status = LED_set_color(LED_COLOR_MAGENTA);
        _POWER_stack_driver_error(led_status, LED_SUCCESS, ERROR_BASE_LED, POWER_ERROR_DRIVER_LED);
        sht3x_status = SHT3X_init();
        _POWER_stack_driver_error(sht3x_status, SHT3X_SUCCESS, ERROR_BASE_SHT30, POWER_ERROR_DRIVER_SHT3X);
        accelerometer_status = ACCELEROMETER_init();
        _POWER_stack_driver_error(accelerometer_status, ACCELEROMETER_SUCCESS, ACCELEROMETER_ERROR_BASE, ACCELEROMETER_POWER_DRIVER_ERROR);
        break;
    case POWER_DOMAIN_GPS:
        // Turn GPS on.
        GPIO_write(&GPIO_GPS_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_GPS;
        // Init attached drivers.
        led_status = LED_set_color(LED_COLOR_YELLOW);
        _POWER_stack_driver_error(led_status, LED_SUCCESS, ERROR_BASE_LED, POWER_ERROR_DRIVER_LED);
        gps_status = GPS_init();
        _POWER_stack_driver_error(gps_status, GPS_SUCCESS, ERROR_BASE_GPS, POWER_ERROR_DRIVER_GPS);
        break;
#ifdef HW2_0
    case POWER_DOMAIN_WIFI:
        // Turn radio on.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 1);
        delay_ms = POWER_ON_DELAY_MS_RADIO;
        // Init attached drivers.
        led_status = LED_set_color(LED_COLOR_RED);
        _POWER_stack_driver_error(led_status, LED_SUCCESS, ERROR_BASE_LED, POWER_ERROR_DRIVER_LED);
        rfe_status = RFE_init();
        _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
        rfe_status = RFE_set_path(RFE_PATH_NONE);
        _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
        wifi_status = WIFI_init();
        _POWER_stack_driver_error(wifi_status, WIFI_SUCCESS, ERROR_BASE_WIFI, POWER_ERROR_DRIVER_WIFI);
        break;
#endif
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
#ifdef HW2_0
        rfe_status = RFE_init();
        _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
        lr11xx_status = LR11XX_init();
        _POWER_stack_driver_error(lr11xx_status, LR11XX_SUCCESS, ERROR_BASE_LR1110, POWER_ERROR_DRIVER_LR11XX);
#else
        s2lp_status = S2LP_init();
        _POWER_stack_driver_error(s2lp_status, S2LP_SUCCESS, ERROR_BASE_S2LP, POWER_ERROR_DRIVER_S2LP);
#endif
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
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    ACCELEROMETER_status_t accelerometer_status = ACCELEROMETER_SUCCESS;
#ifdef HW2_0
    WIFI_status_t wifi_status = WIFI_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
#else
    S2LP_status_t s2lp_status = S2LP_SUCCESS;
#endif
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
        accelerometer_status = ACCELEROMETER_de_init();
        _POWER_stack_driver_error(accelerometer_status, ACCELEROMETER_SUCCESS, ACCELEROMETER_ERROR_BASE, ACCELEROMETER_POWER_DRIVER_ERROR);
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
#ifdef HW2_0
    case POWER_DOMAIN_WIFI:
        // Release attached drivers.
        wifi_status = WIFI_de_init();
        _POWER_stack_driver_error(wifi_status, WIFI_SUCCESS, ERROR_BASE_WIFI, POWER_ERROR_DRIVER_WIFI);
        rfe_status = RFE_de_init();
        _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
        // Turn radio off.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
        break;
#endif
    case POWER_DOMAIN_TCXO:
        // Turn TCXO off.
        GPIO_write(&GPIO_TCXO_POWER_ENABLE, 0);
        break;
    case POWER_DOMAIN_RADIO:
        // Release attached drivers.
#ifdef HW2_0
        lr11xx_status = LR11XX_de_init();
        _POWER_stack_driver_error(lr11xx_status, LR11XX_SUCCESS, ERROR_BASE_LR1110, POWER_ERROR_DRIVER_LR11XX);
        rfe_status = RFE_de_init();
        _POWER_stack_driver_error(rfe_status, RFE_SUCCESS, ERROR_BASE_RFE, POWER_ERROR_DRIVER_RFE);
#else
        s2lp_status = S2LP_de_init();
        _POWER_stack_driver_error(s2lp_status, S2LP_SUCCESS, ERROR_BASE_S2LP, POWER_ERROR_DRIVER_S2LP);
#endif
        // Turn radio off.
        GPIO_write(&GPIO_RF_POWER_ENABLE, 0);
        break;
    default:
        ERROR_stack_add(ERROR_BASE_POWER + POWER_ERROR_DOMAIN);
        goto errors;
    }
errors:
    LED_set_color(LED_COLOR_OFF);
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
