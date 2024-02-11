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

#define TKFX_MODE_CAR
//#define TKFX_MODE_BIKE
//#define TKFX_MODE_HIKING

/*!******************************************************************
 * \struct TKFX_configuration_t
 * \brief Tracker configuration structure.
 *******************************************************************/
typedef struct {
	uint32_t start_detection_threshold_irq;
	uint32_t stop_detection_threshold_seconds;
	uint32_t moving_geoloc_period_seconds;
	uint32_t stopped_geoloc_period_seconds;
	uint32_t monitoring_period_seconds;
} TKFX_configuration_t;

#ifdef TKFX_MODE_CAR
#define TKFX_MODE	0b00
static const TKFX_configuration_t TKFX_CONFIG = {0, 150, 300, 86400, 3600};
#endif
#ifdef TKFX_MODE_BIKE
#define TKFX_MODE	0b01
static const TKFX_configuration_t TKFX_CONFIG = {5, 150, 300, 86400, 3600};
#endif
#ifdef TKFX_MODE_HIKING
#define TKFX_MODE	0b10
static const TKFX_configuration_t TKFX_CONFIG = {5, 60, 600, 86400, 3600};
#endif

#endif /* __MODE_H__ */
