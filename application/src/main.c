/*
 * main.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

// Peripherals.
#include "exti.h"
#include "gpio.h"
#include "i2c_address.h"
#include "iwdg.h"
#include "lptim.h"
#include "nvic.h"
#include "nvic_priority.h"
#include "pwr.h"
#include "rcc.h"
#include "rtc.h"
// Components.
#include "mma865xfc.h"
#include "mma865xfc_configuration.h"
#include "sensors_hw.h"
#include "sht3x.h"
// Utils.
#include "error.h"
#include "types.h"
// Middleware.
#include "analog.h"
#include "cli.h"
#include "gps.h"
#include "power.h"
#include "sigfox_ep_api.h"
#include "sigfox_ep_frames.h"
#include "sigfox_types.h"
#include "sigfox_rc.h"
// Applicative.
#include "at.h"
#include "error_base.h"
#include "tkfx_flags.h"
#include "tkfx_flags_slave.h"
#include "version.h"

/*** MAIN macros ***/

// Monitoring period.
#define TKFX_MONITORING_PERIOD_SECONDS                      3600
// Error stack.
#define TKFX_ERROR_STACK_BLANKING_TIME_SECONDS              86400
// Geolocation.
#define TKFX_GEOLOC_TIMEOUT_SECONDS                         180
#define TKFX_GEOLOC_ALTITUDE_STABILITY_FILTER_MOVING        2
#define TKFX_GEOLOC_ALTITUDE_STABILITY_FILTER_STOPPED       5
// Voltage hysteresis for radio and active mode.
#ifdef TKFX_MODE_SUPERCAPACITOR
#define TKFX_ACTIVE_MODE_ON_STORAGE_VOLTAGE_THRESHOLD_MV    1500
#define TKFX_RADIO_OFF_STORAGE_VOLTAGE_THRESHOLD_MV         1000
#endif
#ifdef TKFX_MODE_BATTERY
#define TKFX_ACTIVE_MODE_ON_STORAGE_VOLTAGE_THRESHOLD_MV    3700
#define TKFX_RADIO_OFF_STORAGE_VOLTAGE_THRESHOLD_MV         3500
#endif
#define TKFX_RADIO_ON_STORAGE_VOLTAGE_THRESHOLD_MV          TKFX_ACTIVE_MODE_ON_STORAGE_VOLTAGE_THRESHOLD_MV

/*** MAIN structures ***/

/*******************************************************************/
typedef enum {
    TKFX_STATE_STARTUP,
    TKFX_STATE_MODE_UPDATE,
    TKFX_STATE_MONITORING,
    TKFX_STATE_GEOLOC,
    TKFX_STATE_ERROR_STACK,
    TKFX_STATE_TASK_CHECK,
    TKFX_STATE_SLEEP,
    TKFX_STATE_LAST
} TKFX_state_t;

/*******************************************************************/
typedef enum {
    TKFX_MODE_ACTIVE,
    TKFX_MODE_LOW_POWER,
    TKFX_MODE_LAST
} TKFX_mode_t;

/*******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned gps_backup_status :1;
        unsigned accelerometer_status :1;
        unsigned tracker_state :1;
        unsigned lse_status :1;
        unsigned moving_flag :1;
        unsigned alarm_flag :1;
        unsigned tracker_mode :2;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} TKFX_status_t;

/*******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned radio_enabled : 1;
        unsigned error_stack_enable :1;
        unsigned geoloc_request :1;
        unsigned monitoring_request :1;
        unsigned por :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} TKFX_flags_t;

/*!******************************************************************
 * \struct TKFX_configuration_t
 * \brief Tracker configuration structure.
 *******************************************************************/
typedef struct {
    uint32_t start_detection_threshold_irq;
    uint32_t stop_detection_threshold_seconds;
    uint32_t moving_geoloc_period_seconds;
    uint32_t stopped_geoloc_period_seconds;
} TKFX_configuration_t;

#ifndef TKFX_MODE_CLI
/*******************************************************************/
typedef struct {
    // State machine.
    TKFX_state_t state;
    TKFX_status_t status;
    TKFX_mode_t mode;
    volatile TKFX_flags_t flags;
    // Monitoring.
    uint32_t monitoring_last_time_seconds;
    uint16_t temperature_tenth_degrees;
    uint8_t humidity_percent;
    uint16_t source_voltage_ten_mv;
    uint16_t storage_voltage_mv;
    // Error stack.
    uint32_t error_stack_last_time_seconds;
    // Tracker algorithm.
    volatile uint32_t motion_irq_count;
    volatile uint32_t motion_irq_last_time_seconds;
    uint32_t geoloc_last_time_seconds;
    NEOM8X_position_t geoloc_position;
} TKFX_context_t;
#endif

/*** MAIN global variables ***/

#ifndef TKFX_MODE_CLI
static TKFX_context_t tkfx_ctx;
#ifdef TKFX_MODE_CAR
static const TKFX_configuration_t TKFX_CONFIG = { 0, 150, 300, 86400 };
#endif
#ifdef TKFX_MODE_BIKE
static const TKFX_configuration_t TKFX_CONFIG = { 2, 150, 300, 86400 };
#endif
#ifdef TKFX_MODE_HIKING
static const TKFX_configuration_t TKFX_CONFIG = { 5, 300, 600, 86400 };
#endif
#endif

/*** MAIN functions ***/

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_rtc_wakeup_timer_irq_callback(void) {
    // Decrement interrupt count.
    if (tkfx_ctx.motion_irq_count > 0) {
        tkfx_ctx.motion_irq_count--;
    }
}
#endif

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_motion_irq_callback(void) {
    // Update variables.
    tkfx_ctx.motion_irq_count++;
    tkfx_ctx.motion_irq_last_time_seconds = RTC_get_uptime_seconds();
}
#endif

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_init_context(void) {
    // Init context.
    tkfx_ctx.state = TKFX_STATE_STARTUP;
    tkfx_ctx.mode = TKFX_MODE_ACTIVE;
    tkfx_ctx.flags.all = 0;
    tkfx_ctx.flags.por = 1;
    tkfx_ctx.flags.radio_enabled = 1;
    tkfx_ctx.flags.error_stack_enable = 1;
    tkfx_ctx.status.all = 0;
#if (defined TKFX_MODE_CAR)
    tkfx_ctx.status.tracker_mode = 0b00;
#elif (defined TKFX_MODE_BIKE)
    tkfx_ctx.status.tracker_mode = 0b01;
#elif (defined TKFX_MODE_HIKING)
    tkfx_ctx.status.tracker_mode = 0b10;
#else
#error "None mode selected"
#endif
    tkfx_ctx.error_stack_last_time_seconds = 0;
    tkfx_ctx.monitoring_last_time_seconds = 0;
    tkfx_ctx.geoloc_last_time_seconds = 0;
    tkfx_ctx.motion_irq_count = 0;
    tkfx_ctx.motion_irq_last_time_seconds = 0;
    // Set motion interrupt callback address.
    SENSORS_HW_set_accelerometer_irq_callback(&_TKFX_motion_irq_callback);
}
#endif

/*******************************************************************/
static void _TKFX_init_hw(void) {
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    RTC_status_t rtc_status = RTC_SUCCESS;
    LPTIM_status_t lptim_status = LPTIM_SUCCESS;
#ifndef TKFX_MODE_DEBUG
    IWDG_status_t iwdg_status = IWDG_SUCCESS;
#endif
    // Init error stack
    ERROR_stack_init();
    // Init memory.
    NVIC_init();
    // Init power module and clock tree.
    PWR_init();
    rcc_status = RCC_init(NVIC_PRIORITY_CLOCK);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init GPIOs.
    GPIO_init();
    POWER_init();
    EXTI_init();
    // Start independent watchdog.
#ifndef TKFX_MODE_DEBUG
    iwdg_status = IWDG_init();
    IWDG_stack_error(ERROR_BASE_IWDG);
    IWDG_reload();
#endif
    // High speed oscillator.
    rcc_status = RCC_switch_to_hsi();
    RCC_stack_error(ERROR_BASE_RCC);
    // Calibrate clocks.
    rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
    RCC_stack_error(ERROR_BASE_RCC);
    // Init RTC.
#ifdef TKFX_MODE_CLI
    rtc_status = RTC_init(NULL, NVIC_PRIORITY_RTC);
#else
    rtc_status = RTC_init(&_TKFX_rtc_wakeup_timer_irq_callback, NVIC_PRIORITY_RTC);
#endif
    RTC_stack_error(ERROR_BASE_RTC);
    // Init delay timer.
    lptim_status = LPTIM_init(NVIC_PRIORITY_DELAY);
    LPTIM_stack_error(ERROR_BASE_LPTIM);
}

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_update_source_storage_voltages(void) {
    // Local variables.
    ANALOG_status_t analog_status = ANALOG_SUCCESS;
    int32_t generic_s32 = 0;
    // Reset data.
    tkfx_ctx.source_voltage_ten_mv = SIGFOX_EP_ERROR_VALUE_SOURCE_VOLTAGE;
    tkfx_ctx.storage_voltage_mv = SIGFOX_EP_ERROR_VALUE_STORAGE_VOLTAGE;
    // Turn ADC on.
    POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_ACTIVE);
    // Perform source voltage measurement.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_SOURCE_VOLTAGE_MV, &generic_s32);
    ANALOG_stack_error(ERROR_BASE_ANALOG);
    if (analog_status == ANALOG_SUCCESS) {
        // Update data.
        tkfx_ctx.source_voltage_ten_mv = (uint16_t) (generic_s32 / 10);
    }
    // Perform storage element voltage measurement.
    analog_status = ANALOG_convert_channel(ANALOG_CHANNEL_STORAGE_VOLTAGE_MV, &generic_s32);
    ANALOG_stack_error(ERROR_BASE_ANALOG);
    if (analog_status == ANALOG_SUCCESS) {
        // Update data.
        tkfx_ctx.storage_voltage_mv = (uint16_t) generic_s32;
    }
    // Turn ADC off.
    POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG);
}
#endif

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_update_temperature_humidity(void) {
    // Local variables.)
    MATH_status_t math_status = MATH_SUCCESS;
    int32_t temperature_tenth_degrees = 0;
    uint32_t temperature_signed_magnitude;
    int32_t humidity = 0;
    SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
    // Reset data.
    tkfx_ctx.temperature_tenth_degrees = SIGFOX_EP_ERROR_VALUE_TEMPERATURE;
    tkfx_ctx.humidity_percent = SIGFOX_EP_ERROR_VALUE_HUMIDITY;
    // Turn sensors on.
    POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
    // Get temperature and humidity from SHT30.
    sht3x_status = SHT3X_get_temperature_humidity(I2C_ADDRESS_SHT30, &temperature_tenth_degrees, &humidity);
    SHT3X_stack_error(ERROR_BASE_SHT30);
    // Check status.
    if (sht3x_status == SHT3X_SUCCESS) {
       // Convert temperature.
       math_status = MATH_integer_to_signed_magnitude(temperature_tenth_degrees, 11, &temperature_signed_magnitude);
       MATH_stack_error(ERROR_BASE_MATH);
       if (math_status == MATH_SUCCESS) {
           tkfx_ctx.temperature_tenth_degrees = (uint16_t) temperature_signed_magnitude;
       }
       tkfx_ctx.humidity_percent = (uint8_t) humidity;
    }
    // Turn sensors off.
    POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
}
#endif

#ifndef TKFX_MODE_CLI
/*******************************************************************/
static void _TKFX_send_sigfox_message(SIGFOX_EP_API_application_message_t* application_message) {
    // Local variables.
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_API_config_t lib_config;
    uint8_t status = 0;
    // Directly exit of the radio is disabled due to low storage element voltage.
    if (tkfx_ctx.flags.radio_enabled == 0) goto errors;
    // Disable motion interrupts.
    SENSORS_HW_disable_accelerometer_interrupt();
    // Library configuration.
    lib_config.rc = &SIGFOX_RC1;
    // Open library.
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    SIGFOX_EP_API_check_status(0);
    // Send message.
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(application_message);
    SIGFOX_EP_API_check_status(0);
    // Close library.
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    SIGFOX_EP_API_check_status(0);
    goto end;
errors:
    SIGFOX_EP_API_close();
    UNUSED(status);
end:
    // Re-enable motion interrupts if enabled.
    if (tkfx_ctx.status.accelerometer_status != 0) {
        // Enable interrupt.
        SENSORS_HW_enable_accelerometer_interrupt();
    }
    return;
}
#endif

#ifndef TKFX_MODE_CLI
/*******************************************************************/
int main(void) {
    // Init board.
    _TKFX_init_context();
    _TKFX_init_hw();
    // Local variables.
    RCC_status_t rcc_status = RCC_SUCCESS;
    MMA865XFC_status_t mma865xfc_status = MMA865XFC_SUCCESS;
    GPS_status_t gps_status = GPS_SUCCESS;
    GPS_acquisition_status_t gps_acquisition_status = GPS_ACQUISITION_SUCCESS;
    SIGFOX_EP_API_application_message_t sigfox_ep_application_message;
    SIGFOX_EP_ul_payload_startup_t sigfox_ep_ul_payload_startup;
    SIGFOX_EP_ul_payload_monitoring_t sigfox_ep_ul_payload_monitoring;
    SIGFOX_EP_ul_payload_geoloc_t sigfox_ep_ul_payload_geoloc;
    SIGFOX_EP_ul_payload_geoloc_timeout_t sigfox_ep_ul_payload_geoloc_timeout;
    ERROR_code_t error_code = 0;
    uint8_t sigfox_ep_ul_payload_error_stack[SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK];
    uint32_t generic_u32_1 = 0;
    uint32_t generic_u32_2 = 0;
    uint8_t generic_u8;
    uint8_t idx = 0;
    // Application message default parameters.
    sigfox_ep_application_message.common_parameters.number_of_frames = 3;
    sigfox_ep_application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
    sigfox_ep_application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
    sigfox_ep_application_message.ul_payload = SIGFOX_NULL;
    sigfox_ep_application_message.ul_payload_size_bytes = 0;
    // Main loop.
    while (1) {
        // Perform state machine.
        switch (tkfx_ctx.state) {
        case TKFX_STATE_STARTUP:
            IWDG_reload();
            // Fill reset reason and software version.
            sigfox_ep_ul_payload_startup.reset_reason = PWR_get_reset_flags();
            sigfox_ep_ul_payload_startup.major_version = GIT_MAJOR_VERSION;
            sigfox_ep_ul_payload_startup.minor_version = GIT_MINOR_VERSION;
            sigfox_ep_ul_payload_startup.commit_index = GIT_COMMIT_INDEX;
            sigfox_ep_ul_payload_startup.commit_id = GIT_COMMIT_ID;
            sigfox_ep_ul_payload_startup.dirty_flag = GIT_DIRTY_FLAG;
            // Clear reset flags.
            PWR_clear_reset_flags();
            // Send startup message.
            sigfox_ep_application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
            sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_startup.frame);
            sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP;
            _TKFX_send_sigfox_message(&sigfox_ep_application_message);
            // Compute next state.
            tkfx_ctx.state = TKFX_STATE_ERROR_STACK;
            break;
        case TKFX_STATE_MONITORING:
            IWDG_reload();
            // Measure related data.
            _TKFX_update_source_storage_voltages();
            _TKFX_update_temperature_humidity();
            // Get clock status.
            rcc_status = RCC_get_status(RCC_CLOCK_LSE, &generic_u8);
            RCC_stack_error(ERROR_BASE_RCC);
            // Update status.
            tkfx_ctx.status.gps_backup_status = GPS_get_backup_voltage();
            tkfx_ctx.status.tracker_state = (tkfx_ctx.mode == TKFX_MODE_LOW_POWER) ? 0b0 : 0b1;
            tkfx_ctx.status.lse_status = (generic_u8 == 0) ? 0b0 : 0b1;
            // Build Sigfox frame.
            sigfox_ep_ul_payload_monitoring.temperature_tenth_degrees = tkfx_ctx.temperature_tenth_degrees;
            sigfox_ep_ul_payload_monitoring.humidity_percent = tkfx_ctx.humidity_percent;
            sigfox_ep_ul_payload_monitoring.source_voltage_ten_mv = tkfx_ctx.source_voltage_ten_mv;
            sigfox_ep_ul_payload_monitoring.storage_voltage_mv = tkfx_ctx.storage_voltage_mv;
            sigfox_ep_ul_payload_monitoring.status = tkfx_ctx.status.all;
            // Send uplink monitoring frame.
            sigfox_ep_application_message.common_parameters.ul_bit_rate = (tkfx_ctx.status.alarm_flag == 0) ? SIGFOX_UL_BIT_RATE_600BPS : SIGFOX_UL_BIT_RATE_100BPS;
            sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_monitoring.frame);
            sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING;
            _TKFX_send_sigfox_message(&sigfox_ep_application_message);
            // Reset request.
            tkfx_ctx.flags.monitoring_request = 0;
            // Compute next state.
            tkfx_ctx.state = TKFX_STATE_ERROR_STACK;
            break;
        case TKFX_STATE_GEOLOC:
            IWDG_reload();
            // Enable backup in active mode and moving condition.
            if ((tkfx_ctx.status.moving_flag != 0) && (tkfx_ctx.mode == TKFX_MODE_ACTIVE)) {
                gps_status = GPS_set_backup_voltage(1);
                GPS_stack_error(ERROR_BASE_GPS);
            }
            // Configure altitude stability filter and reset fix duration.
            generic_u8 = (tkfx_ctx.status.moving_flag == 0) ? TKFX_GEOLOC_ALTITUDE_STABILITY_FILTER_STOPPED : TKFX_GEOLOC_ALTITUDE_STABILITY_FILTER_MOVING;
            generic_u32_1 = 0;
            // Pre-check storage voltage.
            if (tkfx_ctx.storage_voltage_mv > TKFX_ACTIVE_MODE_OFF_STORAGE_VOLTAGE_THRESHOLD_MV) {
                // Turn GPS and analog front-end on to monitor storage element voltage.
                POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG, LPTIM_DELAY_MODE_SLEEP);
                POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS, LPTIM_DELAY_MODE_SLEEP);
                // Get position from GPS.
                gps_status = GPS_get_position(&tkfx_ctx.geoloc_position, generic_u8, TKFX_GEOLOC_TIMEOUT_SECONDS, &generic_u32_1, &gps_acquisition_status);
                GPS_stack_error(ERROR_BASE_GPS);
                // Turn analog front-end and GPS off.
                POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_GPS);
                POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_ANALOG);
            }
            else {
                gps_acquisition_status = GPS_ACQUISITION_ERROR_LOW_STORAGE_VOLTAGE;
            }
            // Disable GPS backup in low power mode or stopped condition.
            if ((tkfx_ctx.status.moving_flag == 0) || (tkfx_ctx.mode == TKFX_MODE_LOW_POWER) || (gps_acquisition_status == GPS_ACQUISITION_ERROR_LOW_STORAGE_VOLTAGE)) {
                gps_status = GPS_set_backup_voltage(0);
                GPS_stack_error(ERROR_BASE_GPS);
            }
            // Compute bit rate according to tracker motion state.
#ifdef TKFX_MODE_HIKING
            sigfox_ep_application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#else
            sigfox_ep_application_message.common_parameters.ul_bit_rate = (tkfx_ctx.status.moving_flag == 0) ? SIGFOX_UL_BIT_RATE_100BPS : SIGFOX_UL_BIT_RATE_600BPS;
#endif
            // Build Sigfox frame.
            if (gps_acquisition_status == GPS_ACQUISITION_SUCCESS) {
                sigfox_ep_ul_payload_geoloc.latitude_degrees = tkfx_ctx.geoloc_position.lat_degrees;
                sigfox_ep_ul_payload_geoloc.latitude_minutes = tkfx_ctx.geoloc_position.lat_minutes;
                sigfox_ep_ul_payload_geoloc.latitude_seconds = tkfx_ctx.geoloc_position.lat_seconds;
                sigfox_ep_ul_payload_geoloc.latitude_north_flag = tkfx_ctx.geoloc_position.lat_north_flag;
                sigfox_ep_ul_payload_geoloc.longitude_degrees = tkfx_ctx.geoloc_position.long_degrees;
                sigfox_ep_ul_payload_geoloc.longitude_minutes = tkfx_ctx.geoloc_position.long_minutes;
                sigfox_ep_ul_payload_geoloc.longitude_seconds = tkfx_ctx.geoloc_position.long_seconds;
                sigfox_ep_ul_payload_geoloc.longitude_east_flag = tkfx_ctx.geoloc_position.long_east_flag;
                sigfox_ep_ul_payload_geoloc.altitude_meters = tkfx_ctx.geoloc_position.altitude;
                sigfox_ep_ul_payload_geoloc.gps_fix_duration_seconds = generic_u32_1;
                // Update message parameters.
                sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_geoloc.frame);
                sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC;
            }
            else {
                sigfox_ep_ul_payload_geoloc_timeout.gps_acquisition_status = gps_acquisition_status;
                sigfox_ep_ul_payload_geoloc_timeout.gps_acquisition_duration_seconds = generic_u32_1;
                // Update message parameters.
                sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_geoloc_timeout.frame);
                sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_TIMEOUT;
            }
            // Send uplink geolocation frame.
            _TKFX_send_sigfox_message(&sigfox_ep_application_message);
            // Reset flag and timer.
            tkfx_ctx.flags.geoloc_request = 0;
            // Compute next state.
            tkfx_ctx.state = TKFX_STATE_ERROR_STACK;
            break;
        case TKFX_STATE_ERROR_STACK:
            IWDG_reload();
            // Check if blanking time elapsed.
            if (tkfx_ctx.flags.error_stack_enable != 0) {
                // Import Sigfox library error stack.
                ERROR_import_sigfox_stack();
                // Check stack.
                if (ERROR_stack_is_empty() == 0) {
                    // Read error stack.
                    for (idx = 0; idx < (SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK >> 1); idx++) {
                        error_code = ERROR_stack_read();
                        sigfox_ep_ul_payload_error_stack[(idx << 1) + 0] = (uint8_t) ((error_code >> 8) & 0x00FF);
                        sigfox_ep_ul_payload_error_stack[(idx << 1) + 1] = (uint8_t) ((error_code >> 0) & 0x00FF);
                    }
                    // Send error stack frame.
                    sigfox_ep_application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
                    sigfox_ep_application_message.ul_payload = (sfx_u8*) (sigfox_ep_ul_payload_error_stack);
                    sigfox_ep_application_message.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK;
                    _TKFX_send_sigfox_message(&sigfox_ep_application_message);
                    // Disable error stack sending.
                    tkfx_ctx.flags.error_stack_enable = 0;
                    tkfx_ctx.error_stack_last_time_seconds = RTC_get_uptime_seconds();
                }
            }
            // Compute next state.
            tkfx_ctx.state = TKFX_STATE_MODE_UPDATE;
            break;
        case TKFX_STATE_MODE_UPDATE:
            IWDG_reload();
            // Measure related data.
            _TKFX_update_source_storage_voltages();
            // Change error value for mode update.
            if (tkfx_ctx.storage_voltage_mv == SIGFOX_EP_ERROR_VALUE_STORAGE_VOLTAGE) {
                tkfx_ctx.storage_voltage_mv = 0;
            }
            // Check storage voltage.
            if ((tkfx_ctx.storage_voltage_mv < TKFX_ACTIVE_MODE_OFF_STORAGE_VOLTAGE_THRESHOLD_MV) || (gps_acquisition_status == GPS_ACQUISITION_ERROR_LOW_STORAGE_VOLTAGE)) {
                tkfx_ctx.mode = TKFX_MODE_LOW_POWER;
            }
            if (tkfx_ctx.storage_voltage_mv > TKFX_ACTIVE_MODE_ON_STORAGE_VOLTAGE_THRESHOLD_MV) {
                tkfx_ctx.mode = TKFX_MODE_ACTIVE;
            }
            // Configure accelerometer according to mode.
            if ((tkfx_ctx.mode == TKFX_MODE_ACTIVE) && ((tkfx_ctx.status.accelerometer_status == 0) || (tkfx_ctx.flags.por != 0))) {
                // Active mode.
                POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
                mma865xfc_status = MMA865XFC_write_configuration(I2C_ADDRESS_MMA8653FC, &(MMA865XFC_ACTIVE_CONFIGURATION[0]), MMA865XFC_ACTIVE_CONFIGURATION_SIZE);
                MMA865XFC_stack_error(ERROR_BASE_MMA8653FC);
                POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
                // Enable interrupt.
                SENSORS_HW_enable_accelerometer_interrupt();
                // Update status.
                tkfx_ctx.status.accelerometer_status = 1;
            }
            if ((tkfx_ctx.mode == TKFX_MODE_LOW_POWER) && ((tkfx_ctx.status.accelerometer_status != 0) || (tkfx_ctx.flags.por != 0))) {
                // Sleep mode.
                POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
                mma865xfc_status = MMA865XFC_write_configuration(I2C_ADDRESS_MMA8653FC, &(MMA865XFC_SLEEP_CONFIGURATION[0]), MMA865XFC_SLEEP_CONFIGURATION_SIZE);
                MMA865XFC_stack_error(ERROR_BASE_MMA8653FC);
                POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
                // Disable interrupt.
                SENSORS_HW_disable_accelerometer_interrupt();
                // Update status.
                tkfx_ctx.status.accelerometer_status = 0;
            }
            // Voltage hysteresis for radio.
            if (tkfx_ctx.storage_voltage_mv < TKFX_RADIO_OFF_STORAGE_VOLTAGE_THRESHOLD_MV) {
                tkfx_ctx.flags.radio_enabled = 0;
            }
            if (tkfx_ctx.storage_voltage_mv > TKFX_RADIO_ON_STORAGE_VOLTAGE_THRESHOLD_MV) {
                tkfx_ctx.flags.radio_enabled = 1;
            }
            // Compute next state.
            tkfx_ctx.state = TKFX_STATE_TASK_CHECK;
            break;
        case TKFX_STATE_TASK_CHECK:
            IWDG_reload();
            // Read uptime.
            generic_u32_1 = RTC_get_uptime_seconds();
            // Periodic monitoring.
            if (generic_u32_1 >= (tkfx_ctx.monitoring_last_time_seconds + TKFX_MONITORING_PERIOD_SECONDS)) {
                // Set request and update last time.
                tkfx_ctx.flags.monitoring_request = 1;
                tkfx_ctx.status.alarm_flag = 0;
                tkfx_ctx.monitoring_last_time_seconds = generic_u32_1;
            }
            // Get current geolocation period.
            generic_u32_2 = ((tkfx_ctx.status.moving_flag == 0) ? TKFX_CONFIG.stopped_geoloc_period_seconds : TKFX_CONFIG.moving_geoloc_period_seconds);
            // Periodic geolocation.
            if (generic_u32_1 >= (tkfx_ctx.geoloc_last_time_seconds + generic_u32_2)) {
                // Check mode.
                if (tkfx_ctx.mode == TKFX_MODE_ACTIVE) {
                    // Set request and update last time.
                    tkfx_ctx.flags.geoloc_request = 1;
                    tkfx_ctx.geoloc_last_time_seconds = generic_u32_1;
                    // Update status.
                    tkfx_ctx.status.alarm_flag = 0;
                }
            }
            // Error stack.
            if (generic_u32_1 >= (tkfx_ctx.error_stack_last_time_seconds + TKFX_ERROR_STACK_BLANKING_TIME_SECONDS)) {
                // Enable error stack message.
                tkfx_ctx.flags.error_stack_enable = 1;
            }
            // Start detection.
            if ((tkfx_ctx.status.moving_flag == 0) && (tkfx_ctx.motion_irq_count > TKFX_CONFIG.start_detection_threshold_irq) && (tkfx_ctx.mode == TKFX_MODE_ACTIVE)) {
                // Set request and update last time.
                tkfx_ctx.flags.monitoring_request = 1;
                tkfx_ctx.monitoring_last_time_seconds = generic_u32_1;
                tkfx_ctx.geoloc_last_time_seconds = generic_u32_1;
                // Update status.
                tkfx_ctx.status.moving_flag = 1;
                tkfx_ctx.status.alarm_flag = 1;
                // Reset interrupts count.
                tkfx_ctx.motion_irq_count = 0;
            }
            else {
                // Stop detection.
                if ((tkfx_ctx.status.moving_flag != 0) && (generic_u32_1 >= (tkfx_ctx.motion_irq_last_time_seconds + TKFX_CONFIG.stop_detection_threshold_seconds)) && (tkfx_ctx.mode == TKFX_MODE_ACTIVE)) {
                    // Set request and update last time.
                    tkfx_ctx.flags.monitoring_request = 1;
                    tkfx_ctx.flags.geoloc_request = 1;
                    tkfx_ctx.monitoring_last_time_seconds = generic_u32_1;
                    tkfx_ctx.geoloc_last_time_seconds = generic_u32_1;
                    // Update status.
                    tkfx_ctx.status.moving_flag = 0;
                    tkfx_ctx.status.alarm_flag = 1;
                    // Reset interrupts count.
                    tkfx_ctx.motion_irq_count = 0;
                }
            }
            // Clear POR flag.
            tkfx_ctx.flags.por = 0;
            // Go to sleep by default.
            tkfx_ctx.state = TKFX_STATE_SLEEP;
            // Check wake-up flags.
            if (tkfx_ctx.flags.geoloc_request != 0) {
                tkfx_ctx.state = TKFX_STATE_GEOLOC;
            }
            if (tkfx_ctx.flags.monitoring_request != 0) {
                tkfx_ctx.state = TKFX_STATE_MONITORING;
            }
            if (tkfx_ctx.state != TKFX_STATE_SLEEP) {
                // Calibrate clocks.
                rcc_status = RCC_calibrate_internal_clocks(NVIC_PRIORITY_CLOCK_CALIBRATION);
                RCC_stack_error(ERROR_BASE_RCC);
                // Reset GPS status for mode update.
                gps_acquisition_status = GPS_ACQUISITION_SUCCESS;

            }
            break;
        case TKFX_STATE_SLEEP:
            // Enter stop mode.
            IWDG_reload();
            PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STOP);
            IWDG_reload();
            // Check wake-up reason.
            tkfx_ctx.state = TKFX_STATE_TASK_CHECK;
            break;
        default:
            // Unknown state.
            tkfx_ctx.state = TKFX_STATE_TASK_CHECK;
            break;
        }
    }
    return 0;
}
#endif

#ifdef TKFX_MODE_CLI
/*******************************************************************/
int main(void) {
    // Local variables.
    CLI_status_t cli_status = CLI_SUCCESS;
    MMA865XFC_status_t mma865xfc_status = MMA865XFC_SUCCESS;
    // Init board.
    _TKFX_init_hw();
    // Configure accelerometer.
    POWER_enable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS, LPTIM_DELAY_MODE_STOP);
    mma865xfc_status = MMA865XFC_write_configuration(I2C_ADDRESS_MMA8653FC, &(MMA865XFC_ACTIVE_CONFIGURATION[0]), MMA865XFC_ACTIVE_CONFIGURATION_SIZE);
    MMA865XFC_stack_error(ERROR_BASE_MMA8653FC);
    POWER_disable(POWER_REQUESTER_ID_MAIN, POWER_DOMAIN_SENSORS);
    // Init command line interface.
    cli_status = CLI_init();
    CLI_stack_error(ERROR_BASE_CLI);
    // Main loop.
    while (1) {
        // Enter sleep mode.
        IWDG_reload();
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        IWDG_reload();
        // Process command line interface.
        cli_status = CLI_process();
        CLI_stack_error(ERROR_BASE_CLI);
    }
    return 0;
}
#endif

