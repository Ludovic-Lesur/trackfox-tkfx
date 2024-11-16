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

#define TKFX_MODE_SUPERCAPACITOR
//#define TKFX_MODE_BATTERY

#ifdef TKFX_MODE_SUPERCAPACITOR
#define TKFX_ACTIVE_MODE_VSTR_MIN_MV    1500
#endif
#ifdef TKFX_MODE_BATTERY
#define TKFX_ACTIVE_MODE_VSTR_MIN_MV    3400
#endif
#define TKFX_GEOLOC_TIMEOUT_SECONDS     180

#define TKFX_MODE_CAR
//#define TKFX_MODE_BIKE
//#define TKFX_MODE_HIKING

#endif /* __MODE_H__ */
