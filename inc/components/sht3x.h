/*
 * sht3x.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef SHT3X_H
#define SHT3X_H

/*** SHT3x macros ***/

#define SHT3X_I2C_ADDRESS	0x44

/*** SHT3x functions ***/

void SHT3X_Init(void);
void SHT3X_PerformMeasurements(unsigned char sht3x_i2c_address);
void SHT3X_GetTemperature(signed char* temperature_degrees);
void SHT3X_GetHumidity(unsigned char* humidity_percent);

#endif /* SHT3X_H */
