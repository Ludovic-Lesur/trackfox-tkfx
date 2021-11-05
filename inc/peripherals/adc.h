/*
 * adc.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef ADC_H
#define ADC_H

/*** ADC structures ***/

typedef enum {
	ADC_DATA_IDX_VSRC_MV = 0,
	ADC_DATA_IDX_VCAP_MV,
	ADC_DATA_IDX_VMCU_MV,
	ADC_DATA_IDX_MAX
} ADC_DataIndex;

/*** ADC functions ***/

void ADC1_Init(void);
void ADC1_Disable(void);
void ADC1_PowerOn(void);
void ADC1_PowerOff(void);
void ADC1_PerformMeasurements(void);
void ADC1_PerformVcapMeasurement(void);
void ADC1_GetData(ADC_DataIndex adc_data_idx, unsigned int* data);
void ADC1_GetTmcuComp2(signed char* tmcu_degrees);
void ADC1_GetTmcuComp1(unsigned char* tmcu_degrees);

#endif /* ADC_H */
