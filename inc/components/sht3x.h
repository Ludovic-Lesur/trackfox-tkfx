/*
 * sht3x.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef SHT3X_H
#define SHT3X_H

/*** SHT3x functions ***/

void SHT3X_Init(void);
void SHT3X_PerformMeasurements(void);
void SHT3X_GetTemperature(unsigned char* temperature_degrees);
void SHT3X_GetHumidity(unsigned char* humidity_percent);

#endif /* SHT3X_H */
