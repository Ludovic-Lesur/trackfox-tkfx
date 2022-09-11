/*
 * mode.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __MODE_H__
#define __MODE_H__

/*** Tracker mode ***/

//#define ATM 		// AT command mode.
#define SSM 		// Start/stop mode.
//#define PM		// Periodic mode.

/*** Debug mode ***/

//#define DEBUG		// Use programming pins for debug purpose if defined.

/*** Error management ***/

#if ((defined ATM && defined SSM) || \
	 (defined ATM && defined PM) || \
	 (defined SSM && defined PM))
#error "Only 1 tracker mode must be selected."
#endif

#endif /* __MODE_H__ */
