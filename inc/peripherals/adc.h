/*
 * adc.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef ADC_H
#define ADC_H

/*** ADC functions ***/

void ADC1_Init(void);
void ADC1_Disable(void);
void ADC1_PowerOn(void);
void ADC1_PowerOff(void);
void ADC1_PerformMeasurements(void);
void ADC1_GetSourceVoltage(unsigned int* source_voltage_mv);
void ADC1_GetSupercapVoltage(unsigned int* supercap_voltage_mv);
void ADC1_GetMcuVoltage(unsigned int* supply_voltage_mv);

#endif /* ADC_H */
