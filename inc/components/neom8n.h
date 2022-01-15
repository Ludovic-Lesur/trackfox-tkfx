/*
 * neom8n.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef NEOM8N_H
#define NEOM8N_H

/*** NEOM8N structures ***/

typedef struct {
	// Latitude.
	unsigned char lat_degrees;
	unsigned char lat_minutes;
	unsigned int lat_seconds; // = (fractionnal part of minutes * 100000).
	unsigned char lat_north_flag; // 0='S', 1='N'.
	// Longitude.
	unsigned char long_degrees;
	unsigned char long_minutes;
	unsigned int long_seconds; // = (fractionnal part of minutes * 100000).
	unsigned char long_east_flag; // 0='O', 1='E'.
	// Altitude.
	unsigned int altitude;
} Position;

typedef enum {
	NEOM8N_SUCCESS,			// Parsing successful and data valid.
	NEOM8N_TIMEOUT			// Parsing failure (= timeout).
} NEOM8N_ReturnCode;

/*** NEOM8N user functions ***/

void NEOM8N_Init(void);
void NEOM8N_SetVbckp(unsigned char vbckp_on);
unsigned char NEOM8N_GetVbckp(void);
NEOM8N_ReturnCode NEOM8N_GetPosition(Position* gps_position, unsigned int timeout_seconds, unsigned int vcap_min_mv, unsigned int* fix_duration_seconds);

/*** NEOM8N utility functions ***/

void NEOM8N_SwitchDmaBuffer(unsigned char lf_flag);

#endif /* NEOM8N_H */
