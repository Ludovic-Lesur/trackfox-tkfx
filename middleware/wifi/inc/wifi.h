/*
 * wifi.h
 *
 *  Created on: 31 mar. 2026
 *      Author: ludo
 */

#ifndef __WIFI_H__
#define __WIFI_H__

#include "error.h"
#include "lr11xx.h"
#include "types.h"

/*** WIFI macros ***/

#define WIFI_MAC_ADDRESS_SIZE_BYTES     LR11XX_WIFI_MAC_ADDRESS_SIZE_BYTES
#define WIFI_MAC_ADDRESS_LIST_SIZE_MAX  LR11XX_WIFI_MAC_ADDRESS_LIST_SIZE_MAX

/*** WIFI structures ***/

/*!******************************************************************
 * \enum WIFI_status_t
 * \brief WIFI driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    WIFI_SUCCESS = 0,
    WIFI_ERROR_NULL_PARAMETER,
    WIFI_ERROR_RADIO_INITIALIZATION,
    // Low level drivers errors.
    WIFI_ERROR_BASE_LR11XX = ERROR_BASE_STEP,
    // Last base value.
    WIFI_ERROR_BASE_LAST = (WIFI_ERROR_BASE_LR11XX + LR11XX_ERROR_BASE_LAST),
} WIFI_status_t;

#ifdef HW2_0

/*!******************************************************************
 * \struct WIFI_channel_info_t
 * \brief WiFi channel informations structure.
 *******************************************************************/
typedef LR11XX_wifi_channel_info_t WIFI_channel_info_t;

/*!******************************************************************
 * \enum WIFI_signal_type_t
 * \brief WiFi signal types list.
 *******************************************************************/
typedef LR11XX_wifi_signal_type_t WIFI_signal_type_t;

/*!******************************************************************
 * \struct WIFI_access_point_t
 * \brief Passive WiFi scan parameters structure.
 *******************************************************************/
typedef LR11XX_wifi_access_point_t WIFI_access_point_t;

/*!******************************************************************
 * \struct WIFI_scan_results_t
 * \brief Passive WiFi scan results structure.
 *******************************************************************/
typedef LR11XX_wifi_scan_results_t WIFI_scan_results_t;

/*** WIFI functions ***/

/*!******************************************************************
 * \fn WIFI_status_t WIFI_init(void)
 * \brief Init WiFi driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
WIFI_status_t WIFI_init(void);

/*!******************************************************************
 * \fn WIFI_status_t WIFI_de_init(void)
 * \brief Release WiFi driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
WIFI_status_t WIFI_de_init(void);

/*!******************************************************************
 * \fnWIFI_status_t WIFI_scan(WIFI_scan_results_t* wifi_scan_results)
 * \brief Perform a passive WiFi scan.
 * \param[in]   none
 * \param[out]  wifi_scan_results: Pointer to the scan results.
 * \retval      Function execution status.
 *******************************************************************/
WIFI_status_t WIFI_scan(WIFI_scan_results_t* wifi_scan_results);

/*******************************************************************/
#define WIFI_exit_error(base) { ERROR_check_exit(wifi_status, WIFI_SUCCESS, base) }

/*******************************************************************/
#define WIFI_stack_error(base) { ERROR_check_stack(wifi_status, WIFI_SUCCESS, base) }

/*******************************************************************/
#define WIFI_stack_exit_error(base, code) { ERROR_check_stack_exit(wifi_status, WIFI_SUCCESS, base, code) }

#endif /* HW2_0 */

#endif /* __WIFI_H__ */
