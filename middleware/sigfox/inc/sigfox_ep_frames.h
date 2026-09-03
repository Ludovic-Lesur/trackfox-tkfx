/*
 * sigfox_ep_frames.h
 *
 *  Created on: 14 nov. 2025
 *      Author: Ludo
 */

#ifndef __SIGFOX_EP_FRAMES_H__
#define __SIGFOX_EP_FRAMES_H__

#include "tkfx_flags.h"
#include "sigfox_types.h"
#include "types.h"

/*** SIGFOX EP FRAMES macros ***/

// Uplink payload sizes.
#define SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP           8
#define SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK       10
#define SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING        7
#define SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC            11
#define SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_TIMEOUT    2
#define SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_ERROR      4
#define SIGFOX_EP_UL_PAYLOAD_SIZE_CONFIGURATION     9
// Error values.
#define SIGFOX_EP_ERROR_VALUE_TEMPERATURE           0x7FF
#define SIGFOX_EP_ERROR_VALUE_HUMIDITY              0xFF
#define SIGFOX_EP_ERROR_VALUE_SOURCE_VOLTAGE        0xFFF
#define SIGFOX_EP_ERROR_VALUE_STORAGE_VOLTAGE       0xFFFF

/*** SIGFOX EP FRAMES structures ***/

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_startup_t
 * \brief Sigfox uplink startup frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP];
    struct {
        unsigned reset_reason :8;
        unsigned major_version :8;
        unsigned minor_version :8;
        unsigned commit_index :8;
        unsigned commit_id :28;
        unsigned dirty_flag :4;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_startup_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_monitoring_t
 * \brief Sigfox uplink monitoring frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING];
    struct {
        unsigned temperature_tenth_degrees :12;
        unsigned humidity_percent :8;
        unsigned source_voltage_ten_mv :12;
        unsigned storage_voltage_mv :16;
        unsigned status :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_monitoring_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_geoloc_t
 * \brief Sigfox uplink geolocation frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC];
    struct {
        unsigned latitude_degrees :8;
        unsigned latitude_minutes :6;
        unsigned latitude_seconds :17;
        unsigned latitude_north_flag :1;
        unsigned longitude_degrees :8;
        unsigned longitude_minutes :6;
        unsigned longitude_seconds :17;
        unsigned longitude_east_flag :1;
        unsigned altitude_meters :16;
        unsigned gps_fix_duration_seconds :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_geoloc_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_geoloc_timeout_t
 * \brief Sigfox uplink geolocation timeout frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_TIMEOUT];
    struct {
        unsigned gps_acquisition_status :8;
        unsigned gps_acquisition_duration_seconds :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_geoloc_timeout_t;

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_geoloc_error_t
 * \brief Sigfox uplink geolocation error frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_GEOLOC_ERROR];
    struct {
        unsigned gps_acquisition_status :8;
        unsigned gps_acquisition_duration_seconds :8;
        unsigned wifi_scan_status :8;
        unsigned wifi_scan_duration_seconds :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_geoloc_error_t;

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_configuration_t
 * \brief Sigfox uplink configuration frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_CONFIGURATION];
    struct {
        unsigned monitoring_period_minutes :8;
        unsigned start_detection_windows :8;
        unsigned start_detection_threshold_irq :8;
        unsigned stop_detection_threshold_minutes :8;
        unsigned geoloc_period_moving_minutes :8;
        unsigned geoloc_period_stopped_hours :8;
        unsigned unused :6;
        unsigned adaptative_tx_power_flag :1;
        unsigned adaptative_ul_bit_rate_flag :1;
        unsigned gps_timeout_seconds :8;
        unsigned gps_altitude_stability_filter_moving :4;
        unsigned gps_altitude_stability_filter_stopped :4;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_configuration_t;
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \enum SIGFOX_EP_dl_op_code_t
 * \brief Sigfox downlink operation codes.
 *******************************************************************/
typedef enum {
    SIGFOX_EP_DL_OP_CODE_NOP = 0,
    SIGFOX_EP_DL_OP_CODE_RESET,
    SIGOFX_EP_DL_OP_CODE_SET_MONITORING_PERIOD,
    SIGFOX_EP_DL_OP_CODE_SET_TRACKING_PARAMETERS,
    SIGFOX_EP_DL_OP_CODE_SET_GPS_SETTINGS,
    SIGFOX_EP_DL_OP_CODE_LAST
} SIGFOX_EP_dl_op_code_t;
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \enum SIGFOX_EP_dl_payload_t
 * \brief Sigfox downlink frames format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
    struct {
        unsigned op_code :8;
        union {
            struct {
                unsigned monitoring_period_minutes :8;
                unsigned unused0 :24;
                unsigned unused1 :24;
            } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) set_monitoring_period;
            struct {
                unsigned start_detection_windows :8;
                unsigned start_detection_threshold_irq :8;
                unsigned stop_detection_threshold_minutes :8;
                unsigned geoloc_period_moving_minutes :8;
                unsigned geoloc_period_stopped_hours :8;
                unsigned unused0 :6;
                unsigned adaptative_tx_power_flag :1;
                unsigned adaptative_ul_bit_rate_flag :1;
                unsigned unused1 :8;
            } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) set_tracking_parameters;
            struct {
                unsigned gps_timeout_seconds :8;
                unsigned gps_altitude_stability_filter_moving :4;
                unsigned gps_altitude_stability_filter_stopped :4;
                unsigned unused0 :8;
                unsigned unused1 :32;
            } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) set_gps_settings;
        };
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_dl_payload_t;
#endif

#endif /* __SIGFOX_EP_FRAMES_H__ */
