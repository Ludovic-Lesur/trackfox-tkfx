/*
 * tkfx_flags_slave.h
 *
 *  Created on: 03 jan. 2026
 *      Author: Ludo
 */

#ifndef __TKFX_FLAGS_SLAVE_H__
#define __TKFX_FLAGS_SLAVE_H__

#include "tkfx_flags.h"

/*** Slave compilation flags ***/

#ifdef TKFX_MODE_SUPERCAPACITOR
#define TKFX_STORAGE_VOLTAGE_MV_MAX                         2700
#define TKFX_MODE_ACTIVE_STORAGE_VOLTAGE_THRESHOLD_MV       1500
#define TKFX_MODE_LOW_POWER_STORAGE_VOLTAGE_THRESHOLD_MV    1000
#endif
#ifdef TKFX_MODE_BATTERY
#define TKFX_STORAGE_VOLTAGE_MV_MAX                         4200
#define TKFX_MODE_ACTIVE_STORAGE_VOLTAGE_THRESHOLD_MV       3700
#define TKFX_MODE_LOW_POWER_STORAGE_VOLTAGE_THRESHOLD_MV    3500
#endif
#define TKFX_MODE_SWITCH_HYSTERESIS_MV                      100

#endif /* __TKFX_FLAGS_SLAVE_H__ */
