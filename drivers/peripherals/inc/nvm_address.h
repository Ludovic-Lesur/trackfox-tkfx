/*
 * nvm_address.h
 *
 *  Created on: 12 nov. 2024
 *      Author: Ludo
 */

#ifndef __NVM_ADDRESS_H__
#define __NVM_ADDRESS_H__

#include "sigfox_types.h"

/*!******************************************************************
 * \enum NVM_address_mapping_t
 * \brief NVM address mapping.
 *******************************************************************/
typedef enum {
    // Sigfox library.
    NVM_ADDRESS_SIGFOX_EP_ID = 0,
    NVM_ADDRESS_SIGFOX_EP_KEY = (NVM_ADDRESS_SIGFOX_EP_ID + SIGFOX_EP_ID_SIZE_BYTES),
    NVM_ADDRESS_SIGFOX_EP_LIB_DATA = (NVM_ADDRESS_SIGFOX_EP_KEY + SIGFOX_EP_KEY_SIZE_BYTES),
    // Tracker configuration.
    NVM_ADDRESS_MONITORING_PERIOD_MINUTES = (NVM_ADDRESS_SIGFOX_EP_LIB_DATA + SIGFOX_NVM_DATA_SIZE_BYTES),
    NVM_ADDRESS_START_DETECTION_WINDOWS,
    NVM_ADDRESS_START_DETECTION_THRESHOLD_IRQ,
    NVM_ADDRESS_STOP_DETECTION_THRESHOLD_MINUTES,
    NVM_ADDRESS_GEOLOC_PERIOD_MOVING_MINUTES,
    NVM_ADDRESS_GEOLOC_PERIOD_STOPPED_HOURS,
    NVM_ADDRESS_ADAPTATIVE_TX_POWER_FLAG,
    NVM_ADDRESS_ADAPTATIVE_UL_BIT_RATE_FLAG,
    NVM_ADDRESS_GPS_TIMEOUT_SECONDS,
    NVM_ADDRESS_GPS_ALTITUDE_STABILITY_FILTER_MOVING,
    NVM_ADDRESS_GPS_ALTITUDE_STABILITY_FILTER_STOPPED
} NVM_address_mapping_t;

#endif /* __NVM_ADDRESS_H__ */
