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
#define TKFX_ACTIVE_MODE_OFF_VSTR_THRESHOLD_MV  1500
#endif
#ifdef TKFX_MODE_BATTERY
#define TKFX_ACTIVE_MODE_OFF_VSTR_THRESHOLD_MV  3700
#endif

#endif /* __TKFX_FLAGS_SLAVE_H__ */
