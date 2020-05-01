/*
 * mode.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef MODE_H
#define MODE_H

/*** Tracker mode ***/

//#define ATM 		// AT command mode.
#define SSM 		// Start/stop mode.
//#define MS 		// Manual mode.

/*** Debug mode ***/

#define DEBUG		// Use programming pins for debug purpose if defined.

/*** Error management ***/

#if ((defined ATM && defined SSM) || \
	 (defined ATM && defined MS) || \
	 (defined SSM && defined MS))
#error "Only 1 tracker mode must be selected."
#endif

#endif /* MODE_H */
