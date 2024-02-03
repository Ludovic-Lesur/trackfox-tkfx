/*
 * mode.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __MODE_H__
#define __MODE_H__

#include "types.h"

/*** Board modes ***/

//#define ATM
//#define DEBUG

/*** Board options ***/

#define TKFX_VSTR_VOLTAGE_DIVIDER_RATIO		1
#define TKFX_ACTIVE_MODE_VSTR_MIN_MV		1500
#define TKFX_GEOLOC_TIMEOUT_SECONDS			180

/*!******************************************************************
 * \struct TKFX_configuration_t
 * \brief Tracker configuration structure.
 *******************************************************************/
typedef struct {
	uint32_t start_detection_threshold_irq; // Number of accelerometer interrupts required to trigger start condition.
	uint32_t stop_detection_threshold_seconds; // Delay required to trigger stop condition.
	uint32_t moving_geoloc_period_seconds;
	uint32_t stopped_geoloc_period_seconds;
	uint32_t monitoring_period_seconds;
} TKFX_configuration_t;

static const TKFX_configuration_t TKFX_CONFIG = {0, 300, 600, 86400, 3600}; // Car.
//static const TKFX_configuration_t TKFX_CONFIG = {5, 60, 1200, 86400, 3600}; // Hiking.
//static const TKFX_configuration_t TKFX_CONFIG = {5, 300, 600, 86400, 3600}; // Bike.

#endif /* __MODE_H__ */
