/*
 * gps.c
 *
 *  Created on: 21 aug. 2024
 *      Author: Ludo
 */

#include "gps.h"

#include "analog.h"
#include "error.h"
#include "error_base.h"
#include "iwdg.h"
#include "maxm10x.h"
#include "neom8x.h"
#include "pwr.h"
#include "rtc.h"
#include "tkfx_flags.h"
#include "tkfx_flags_slave.h"
#include "types.h"

/*** GPS local macros ***/

#define GPS_TIMEOUT_SECONDS                     10

#ifdef HW2_0

#define gps_module_status                       maxm10x_status
#define GPS_MODULE_SUCCESS                      MAXM10X_SUCCESS
#define GPS_MODULE_exit_error(void)             MAXM10X_exit_error(GPS_ERROR_BASE_MAXM10X)
#define GPS_MODULE_stack_error(void)            MAXM10X_stack_error(ERROR_BASE_GPS + GPS_ERROR_BASE_MAXM10X)

#define GPS_MODULE_ACQUISITION_STATUS_FAIL      MAXM10X_ACQUISITION_STATUS_FAIL
#define GPS_MODULE_ACQUISITION_STATUS_FOUND     MAXM10X_ACQUISITION_STATUS_FOUND
#define GPS_MODULE_ACQUISITION_STATUS_STABLE    MAXM10X_ACQUISITION_STATUS_FOUND
#define GPS_MODULE_GPS_DATA_POSITION            MAXM10X_GPS_DATA_POSITION

#define GPS_MODULE_init                         MAXM10X_init
#define GPS_MODULE_de_init                      MAXM10X_de_init
#define GPS_MODULE_start_acquisition            MAXM10X_start_acquisition
#define GPS_MODULE_stop_acquisition             MAXM10X_stop_acquisition
#define GPS_MODULE_process                      MAXM10X_process
#define GPS_MODULE_get_position                 MAXM10X_get_position
#define GPS_MODULE_set_backup_voltage           MAXM10X_set_backup_voltage
#define GPS_MODULE_get_backup_voltage           MAXM10X_get_backup_voltage

#else

#define gps_module_status                       neom8x_status
#define GPS_MODULE_SUCCESS                      NEOM8X_SUCCESS
#define GPS_MODULE_exit_error(void)             NEOM8X_exit_error(GPS_ERROR_BASE_NEOM8N)
#define GPS_MODULE_stack_error(void)            NEOM8X_stack_error(ERROR_BASE_GPS + GPS_ERROR_BASE_NEOM8N)

#define GPS_MODULE_ACQUISITION_STATUS_FAIL      NEOM8X_ACQUISITION_STATUS_FAIL
#define GPS_MODULE_ACQUISITION_STATUS_FOUND     NEOM8X_ACQUISITION_STATUS_FOUND
#define GPS_MODULE_ACQUISITION_STATUS_STABLE    NEOM8X_ACQUISITION_STATUS_FOUND
#define GPS_MODULE_GPS_DATA_POSITION            NEOM8X_GPS_DATA_POSITION

#define GPS_MODULE_init                         NEOM8X_init
#define GPS_MODULE_de_init                      NEOM8X_de_init
#define GPS_MODULE_start_acquisition            NEOM8X_start_acquisition
#define GPS_MODULE_stop_acquisition             NEOM8X_stop_acquisition
#define GPS_MODULE_process                      NEOM8X_process
#define GPS_MODULE_get_position                 NEOM8X_get_position
#define GPS_MODULE_set_backup_voltage           NEOM8X_set_backup_voltage
#define GPS_MODULE_get_backup_voltage           NEOM8X_get_backup_voltage

#endif /* HW2.0 */

/*** GPS local structures ***/

#ifdef HW2_0
typedef MAXM10X_status_t GPS_MODULE_status_t;
typedef MAXM10X_acquisition_t GPS_MODULE_acquisition_t;
typedef MAXM10X_acquisition_status_t GPS_MODULE_acquisition_status_t;
#else
typedef NEOM8X_status_t GPS_MODULE_status_t;
typedef NEOM8X_acquisition_t GPS_MODULE_acquisition_t;
typedef NEOM8X_acquisition_status_t GPS_MODULE_acquisition_status_t;
#endif

/*******************************************************************/
typedef struct {
    volatile uint8_t process_flag;
    GPS_MODULE_acquisition_status_t acquisition_status;
} GPS_context_t;

/*** GPS local global variables ***/

static GPS_context_t gps_ctx = {
    .process_flag = 0,
    .acquisition_status = GPS_MODULE_ACQUISITION_STATUS_FAIL
};

/*** GPS local functions ***/

/*******************************************************************/
static void _GPS_process_callback(void) {
    // Set local flag.
    gps_ctx.process_flag = 1;
}

/*******************************************************************/
static void _GPS_completion_callback(GPS_MODULE_acquisition_status_t acquisition_status) {
    // Update global variable.
    gps_ctx.acquisition_status = acquisition_status;
}

/*** GPS functions ***/

/*******************************************************************/
GPS_status_t GPS_init(void) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    GPS_MODULE_status_t gps_module_status = GPS_MODULE_SUCCESS;
    // Init GPS module.
    gps_module_status = GPS_MODULE_init();
    GPS_MODULE_exit_error();
errors:
    return status;
}

/*******************************************************************/
GPS_status_t GPS_de_init(void) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    GPS_MODULE_status_t gps_module_status = GPS_MODULE_SUCCESS;
    // Init GPS module.
    gps_module_status = GPS_MODULE_de_init();
    GPS_MODULE_stack_error();
    return status;
}

/*******************************************************************/
GPS_status_t GPS_get_position(GPS_position_t* gps_position, uint8_t altitude_stability_threshold, uint32_t timeout_seconds, uint32_t* acquisition_duration_seconds, GPS_acquisition_status_t* acquisition_status) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    GPS_MODULE_status_t gps_module_status = GPS_MODULE_SUCCESS;
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    GPS_MODULE_acquisition_t gps_acquisition;
    GPS_MODULE_acquisition_status_t expected_status = (altitude_stability_threshold == 0) ? GPS_MODULE_ACQUISITION_STATUS_FOUND : GPS_MODULE_ACQUISITION_STATUS_STABLE;
    uint32_t uptime = RTC_get_uptime_seconds();
    uint32_t start_time = uptime;
    int32_t storage_voltage_voltage_mv = 0;
    uint8_t callback_flag = 0;
    // Check parameters.
    if ((acquisition_duration_seconds == NULL) || (acquisition_status == NULL)) {
        status = GPS_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Reset output data.
    (*acquisition_duration_seconds) = 0;
    (*acquisition_status) = GPS_ACQUISITION_ERROR_TIMEOUT;
    gps_ctx.acquisition_status = GPS_MODULE_ACQUISITION_STATUS_FAIL;
    // Configure GPS acquisition.
    gps_acquisition.gps_data = GPS_MODULE_GPS_DATA_POSITION;
    gps_acquisition.completion_callback = &_GPS_completion_callback;
    gps_acquisition.process_callback = &_GPS_process_callback;
    gps_acquisition.altitude_stability_threshold = altitude_stability_threshold;
    // Start acquisition.
    gps_module_status = GPS_MODULE_start_acquisition(&gps_acquisition);
    GPS_MODULE_exit_error();
    // Processing loop.
    while (uptime < (start_time + timeout_seconds)) {
        // Ensure RTC is running.
        if (RTC_get_uptime_seconds() > uptime) {
            // Update time and reload watchdog.
            uptime = RTC_get_uptime_seconds();
            IWDG_reload();
        }
        // Enter sleep mode.
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        // Update acquisition duration.
        (*acquisition_duration_seconds) = (uptime - start_time);
        // Check flag.
        if (gps_ctx.process_flag != 0) {
            // Update flags.
            gps_ctx.process_flag = 0;
            callback_flag = 1;
            // Process driver.
            gps_module_status = GPS_MODULE_process();
            GPS_MODULE_exit_error();
            // Check STORAGE_VOLTAGE voltage.
            analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_STORAGE_VOLTAGE_MV, &storage_voltage_voltage_mv);
            ANALOG_exit_error(GPS_ERROR_BASE_ANALOG);
            // Check threshold.
            if (storage_voltage_voltage_mv < TKFX_MODE_ACTIVE_STORAGE_VOLTAGE_THRESHOLD_MV) {
                (*acquisition_status) = GPS_ACQUISITION_ERROR_LOW_STORAGE_VOLTAGE;
                break;
            }
        }
        // Check acquisition status.
        if (gps_ctx.acquisition_status == expected_status) break;
        // Exit if process callback has never been called.
        if ((callback_flag == 0) && (uptime > (start_time + GPS_TIMEOUT_SECONDS))) {
            status = GPS_ERROR_PROCESS_CALLBACK;
            goto errors;
        }
    }
    gps_module_status = GPS_MODULE_stop_acquisition();
    GPS_MODULE_exit_error();
    // Check status.
    if (gps_ctx.acquisition_status != GPS_MODULE_ACQUISITION_STATUS_FAIL) {
        // Read data.
        gps_module_status = GPS_MODULE_get_position(gps_position);
        GPS_MODULE_exit_error();
        // Update status.
        (*acquisition_status) = GPS_ACQUISITION_SUCCESS;
    }
errors:
    GPS_MODULE_stop_acquisition();
    return status;
}

/*******************************************************************/
GPS_status_t GPS_set_backup_voltage(uint8_t state) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    GPS_MODULE_status_t gps_module_status = GPS_MODULE_SUCCESS;
    // Set backup state.
    gps_module_status = GPS_MODULE_set_backup_voltage(state);
    GPS_MODULE_exit_error();
errors:
    return status;
}

/*******************************************************************/
GPS_status_t GPS_get_backup_voltage(uint8_t* state) {
    // Local variables.
    GPS_status_t status = GPS_SUCCESS;
    GPS_MODULE_status_t gps_module_status = GPS_MODULE_SUCCESS;
    // Get backup state.
    gps_module_status = GPS_MODULE_get_backup_voltage(state);
    GPS_MODULE_exit_error();
errors:
    return status;
}
