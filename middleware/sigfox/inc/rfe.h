/*
 * rfe.h
 *
 *  Created on: 19 mar. 2026
 *      Author: Ludo
 */

#ifndef __RFE_H__
#define __RFE_H__

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "error.h"
#include "lr11xx.h"
#include "types.h"

/*** RFE structures ***/

/*!******************************************************************
 * \enum RFE_status_t
 * \brief Radio front-end driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    RFE_SUCCESS = 0,
    RFE_ERROR_PATH,
    // Low level drivers errors.
    RFE_ERROR_BASE_LR11XX = ERROR_BASE_STEP,
    // Last base value.
    RFE_ERROR_BASE_LAST = (RFE_ERROR_BASE_LR11XX + LR11XX_ERROR_BASE_LAST)
} RFE_status_t;

#ifdef HW2_0

/*!******************************************************************
 * \enum RFE_path_t
 * \brief Radio front-end paths list.
 *******************************************************************/
typedef enum {
    RFE_PATH_OFF = 0,
    RFE_PATH_NONE,
    RFE_PATH_TX_LOW_POWER,
    RFE_PATH_TX_HIGH_POWER,
#ifdef SIGFOX_EP_BIDIRECTIONAL
    RFE_PATH_RX_BYPASS,
    RFE_PATH_RX_LNA,
#endif
    RFE_PATH_LAST
} RFE_path_t;

/*** RFE functions ***/

/*!******************************************************************
 * \fn RFE_status_t RFE_init(void)
 * \brief Init radio front-end interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RFE_status_t RFE_init(void);

/*!******************************************************************
 * \fn RFE_status_t RFE_de_init(void)
 * \brief Release radio front-end interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RFE_status_t RFE_de_init(void);

/*!******************************************************************
 * \fn RFE_status_t RFE_set_path(RFE_path_t radio_path)
 * \brief Select active radio path.
 * \param[in]   radio_path: Radio line to select.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RFE_status_t RFE_set_path(RFE_path_t radio_path);

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \fn RFE_status_t RFE_get_rssi(LR11XX_rssi_t rssi_type, int16_t* rssi_dbm)
 * \brief Get calibrated RSSI at board connector.
 * \param[in]   rssi_type: RSSI type to read.
 * \param[out]  rssi_dbm: Pointer to signed 16-bits value that will contain the RSSI in dBm.
 * \retval      Function execution status.
 *******************************************************************/
RFE_status_t RFE_get_rssi(LR11XX_rssi_t rssi_type, int16_t* rssi_dbm);
#endif

/*******************************************************************/
#define RFE_exit_error(base) { ERROR_check_exit(rfe_status, RFE_SUCCESS, base) }

/*******************************************************************/
#define RFE_stack_error(base) { ERROR_check_stack(rfe_status, RFE_SUCCESS, base) }

/*******************************************************************/
#define RFE_stack_exit_error(base, code) { ERROR_check_stack_exit(rfe_status, RFE_SUCCESS, base, code) }

#endif /* HW2_0 */

#endif /* __RFE_H__ */
