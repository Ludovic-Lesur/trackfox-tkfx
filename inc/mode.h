/*
 * mode.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __MODE_H__
#define __MODE_H__

#include "types.h"

/*** Tracker mode ***/

//#define ATM 		// AT command mode.
#define SSM 		// Start/stop mode.
//#define PM		// Periodic mode.

/*** Tracker configuration ***/

/*!******************************************************************
 * \enum TKFX_configuration_t
 * \brief Tracker configuration structure.
 *******************************************************************/
typedef struct {
	uint32_t vcap_min_mv;
	uint32_t geoloc_timeout_seconds;
#ifdef SSM
	uint32_t start_detection_threshold_irq; // Number of accelerometer interrupts required to trigger start condition (set to 0 to disable filter).
	uint32_t stop_detection_threshold_seconds; // Delay required to trigger stop condition.
	uint32_t keep_alive_period_seconds;
	uint32_t inactivity_geoloc_enabled; // If non zero, force a GPS fix after an inactivity period.
	uint32_t inactivity_threshold_seconds;
#endif
#ifdef PM
	uint32_t geoloc_period_seconds;
#endif
} TKFX_configuration_t;

#ifdef SSM
//static const TKFX_configuration_t TKFX_CONFIG = {1500, 180, 0, 300, 3600, 1, 86400}; // Car tracking configuration.
static const TKFX_configuration_t TKFX_CONFIG = {1500, 180, 5, 60, 3600, 0, 86400}; // Hiking configuration.
#endif
#ifdef PM
static const TKFX_configuration_t TKFX_CONFIG = {1500, 180, 300}; // Bike tracking configuration.
#endif

/*** Debug mode ***/

//#define DEBUG		// Use programming pins for debug purpose if defined.

/*** Error management ***/

#if ((defined ATM && defined SSM) || \
	 (defined ATM && defined PM) || \
	 (defined SSM && defined PM))
#error "Only 1 tracker mode must be selected."
#endif

#endif /* __MODE_H__ */
