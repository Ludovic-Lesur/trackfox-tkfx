/*
 * sht3x.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef SHT3X_H
#define SHT3X_H

/*** SHT3x functions ***/

void SHT3X_init(void);
void SHT3X_perform_measurements(void);
void SHT3X_get_temperature_comp1(unsigned char* temperature_degrees);
void SHT3X_get_temperature_comp2(signed char* temperature_degrees);
void SHT3X_get_humidity(unsigned char* humidity_percent);

#endif /* SHT3X_H */
