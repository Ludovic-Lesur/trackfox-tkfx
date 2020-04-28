/*
 * dlk.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludovic
 */

#ifndef DLK_H
#define DLK_H

/*** DLK structures ***/

typedef struct {
	// TBD.
} DLK_Parameters;

/*** DLK functions ***/

void DLK_Read(DLK_Parameters* config);
void DLK_Write(unsigned char* downlink_data);

#endif /* DLK_H */
