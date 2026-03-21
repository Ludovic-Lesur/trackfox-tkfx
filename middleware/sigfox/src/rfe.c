/*
 * rfe.c
 *
 *  Created on: 19 mar. 2026
 *      Author: Ludo
 */

#include "rfe.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "mcu_mapping.h"
#include "lr11xx.h"
#include "types.h"

#ifdef HW2_0

/*** RFE local global variables ***/

#ifdef SIGFOX_EP_BIDIRECTIONAL
static const int16_t RFE_RSSI_OFFSET_DB[RFE_PATH_LAST] = { 0, 0, (-2), 15 };
#endif
static RFE_path_t rfe_current_path = RFE_PATH_NONE;

/*** RFE functions ***/

/*******************************************************************/
RFE_status_t RFE_init(void) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    // Configure GPIOs.
    GPIO_configure(&GPIO_RF_SW_V1, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_RF_SW_V1, 0);
    GPIO_configure(&GPIO_RF_SW_V2, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_RF_SW_V2, 0);
    GPIO_configure(&GPIO_RF_LNA_BYPASS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_write(&GPIO_RF_LNA_BYPASS, 0);
    return status;
}

/*******************************************************************/
RFE_status_t RFE_de_init(void) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    // Set all pins to output low.
    rfe_status = RFE_set_path(RFE_PATH_NONE);
    RFE_stack_error(ERROR_BASE_RFE);
    return status;
}

/*******************************************************************/
RFE_status_t RFE_set_path(RFE_path_t radio_path) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    switch (radio_path) {
    case RFE_PATH_NONE:
        GPIO_write(&GPIO_RF_SW_V1, 0);
        GPIO_write(&GPIO_RF_SW_V2, 0);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 0);
        break;
    case RFE_PATH_TX_LOW_POWER:
        GPIO_write(&GPIO_RF_SW_V1, 1);
        GPIO_write(&GPIO_RF_SW_V2, 1);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 1);
        break;
    case RFE_PATH_TX_HIGH_POWER:
        GPIO_write(&GPIO_RF_SW_V1, 0);
        GPIO_write(&GPIO_RF_SW_V2, 1);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 1);
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case RFE_PATH_RX_BYPASS:
        GPIO_write(&GPIO_RF_SW_V1, 1);
        GPIO_write(&GPIO_RF_SW_V2, 0);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 1);
        break;
    case RFE_PATH_RX_LNA:
        GPIO_write(&GPIO_RF_SW_V1, 1);
        GPIO_write(&GPIO_RF_SW_V2, 0);
        GPIO_write(&GPIO_RF_LNA_BYPASS, 0);
        break;
#endif
    default:
        status = RFE_ERROR_PATH;
        goto errors;
    }
    rfe_current_path = radio_path;
errors:
    return status;
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RFE_status_t RFE_get_rssi(LR11XX_rssi_t rssi_type, int16_t* rssi_dbm) {
    // Local variables.
    RFE_status_t status = RFE_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    // Read raw RSSI.
    lr11xx_status = LR11XX_get_rssi(rssi_type, rssi_dbm);
    LR11XX_exit_error(RFE_ERROR_BASE_LR11XX);
    // Apply calibration gain.
    (*rssi_dbm) -= RFE_RSSI_OFFSET_DB[rfe_current_path];
errors:
    return status;
}
#endif

#endif
