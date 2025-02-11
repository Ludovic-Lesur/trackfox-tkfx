/*
 * tkfx_flags.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __TKFX_FLAGS_H__
#define __TKFX_FLAGS_H__

#include "types.h"

/*** Board modes ***/

#define TKFX_MODE_CAR
//#define TKFX_MODE_BIKE
//#define TKFX_MODE_HIKING

#define TKFX_MODE_BATTERY
//#define TKFX_MODE_SUPERCAPACITOR

//#define TKFX_MODE_CLI
//#define TKFX_MODE_DEBUG

/*** Board parameters ***/

#ifdef TKFX_MODE_SUPERCAPACITOR
#define TKFX_ACTIVE_MODE_OFF_VSTR_THRESHOLD_MV  1500
#endif
#ifdef TKFX_MODE_BATTERY
#define TKFX_ACTIVE_MODE_OFF_VSTR_THRESHOLD_MV  3700
#endif

#endif /* __TKFX_FLAGS_H__ */
