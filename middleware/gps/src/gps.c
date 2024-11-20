/*
 * gps.c
 *
 *  Created on: 21 aug. 2024
 *      Author: Ludo
 */

#include "gps.h"

#include "analog.h"
#include "error.h"
#include "iwdg.h"
#include "mode.h"
#include "neom8x.h"
#include "pwr.h"
#include "rtc.h"
#include "types.h"

/*** GPS local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint8_t process_flag;
    NEOM8X_acquisition_status_t acquisition_status;
} GPS_context_t;

/*** GPS local global variables ***/

static GPS_context_t gps_ctx;

/*** GPS local functions ***/

/*******************************************************************/
static void _GPS_process_callback(void) {
    // Set local flag.
    gps_ctx.process_flag = 1;
}

/*******************************************************************/
static void _GPS_completion_callback(NEOM8X_acquisition_status_t acquisition_status) {
    // Update global variable.
    gps_ctx.acquisition_status = acquisition_status;
}

/*** GPS functions ***/

/*******************************************************************/
GPS_status_t GPS_init(void) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    NEOM8X_status_t neom8x_status = NEOM8X_SUCCESS;
    // Init GPS module.
    neom8x_status = NEOM8X_init();
    NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
errors:
    return status;
}

/*******************************************************************/
GPS_status_t GPS_de_init(void) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    NEOM8X_status_t neom8x_status = NEOM8X_SUCCESS;
    // Init GPS module.
    neom8x_status = NEOM8X_de_init();
    NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
errors:
    return status;
}

/*******************************************************************/
GPS_status_t GPS_get_position(GPS_position_t* gps_position, uint8_t altitude_stability_threshold, uint32_t timeout_seconds, uint32_t* acquisition_duration_seconds, GPS_acquisition_status_t* acquisition_status) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    NEOM8X_status_t neom8x_status = NEOM8X_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    NEOM8X_acquisition_t gps_acquisition;
    NEOM8X_acquisition_status_t expected_status = (altitude_stability_threshold == 0) ? NEOM8X_ACQUISITION_STATUS_FOUND : NEOM8X_ACQUISITION_STATUS_STABLE;
    uint32_t start_time = RTC_get_uptime_seconds();
    int32_t vstr_voltage_mv = 0;
    // Check parameters.
    if ((acquisition_duration_seconds == NULL) || (acquisition_status == NULL)) {
        status = GPS_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Reset output data.
    (*acquisition_duration_seconds) = 0;
    (*acquisition_status) = GPS_ACQUISITION_ERROR_TIMEOUT;
    gps_ctx.acquisition_status = NEOM8X_ACQUISITION_STATUS_FAIL;
    // Configure GPS acquisition.
    gps_acquisition.gps_data = NEOM8X_GPS_DATA_POSITION;
    gps_acquisition.completion_callback = &_GPS_completion_callback;
    gps_acquisition.process_callback = &_GPS_process_callback;
    gps_acquisition.altitude_stability_threshold = altitude_stability_threshold;
    // Start acquisition.
    neom8x_status = NEOM8X_start_acquisition(&gps_acquisition);
    NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
    // Processing loop.
    while (RTC_get_uptime_seconds() < (start_time + timeout_seconds)) {
        // Enter sleep mode.
        IWDG_reload();
        PWR_enter_sleep_mode();
        // Update acquisition duration.
        (*acquisition_duration_seconds) = (RTC_get_uptime_seconds() - start_time);
        // Check flag.
        if (gps_ctx.process_flag != 0) {
            neom8x_status = NEOM8X_process();
            NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
            // Check VSTR voltage.
            analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_VSTR_MV, &vstr_voltage_mv);
            ANALOG_exit_error(GPS_ERROR_BASE_ANALOG);
            // Check threshold.
            if (vstr_voltage_mv < TKFX_ACTIVE_MODE_VSTR_MIN_MV) {
                (*acquisition_status) = GPS_ACQUISITION_ERROR_VSTR_THRESHOLD;
                break;
            }
        }
        // Check acquisition status.
        if (gps_ctx.acquisition_status == expected_status) break;
    }
    neom8x_status = NEOM8X_stop_acquisition();
    NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
    // Check status.
    if (gps_ctx.acquisition_status != NEOM8X_ACQUISITION_STATUS_FAIL) {
        // Read data.
        neom8x_status = NEOM8X_get_position(gps_position);
        NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
        // Update status.
        (*acquisition_status) = GPS_ACQUISITION_SUCCESS;
    }
    return status;
errors:
    NEOM8X_stop_acquisition();
    return status;
}

/*******************************************************************/
GPS_status_t GPS_set_backup_voltage(uint8_t state) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
#ifdef HW1_1
    NEOM8X_status_t neom8x_status = NEOM8X_SUCCESS;
    // Set backup state.
    neom8x_status = NEOM8X_set_backup_voltage(state);
    NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N);
errors:
#else
    UNUSED(state);
#endif
    return status;
}

/*******************************************************************/
uint8_t GPS_get_backup_voltage(void) {
#ifdef HW1_0
    return 1;
#else
    return NEOM8X_get_backup_voltage();
#endif
}
