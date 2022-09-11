/*
 * rcc.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __RCC_H__
#define __RCC_H__

/*** RCC macros ***/

#define RCC_LSI_FREQUENCY_HZ	38000
#define RCC_LSE_FREQUENCY_HZ	32768
#define RCC_MSI_FREQUENCY_KHZ	65
#define RCC_HSI_FREQUENCY_KHZ	16000

/*** RCC functions ***/

void RCC_init(void);
unsigned int RCC_get_sysclk_khz(void);
unsigned char RCC_switch_to_msi(void);
unsigned char RCC_switch_to_hsi(void);
unsigned char RCC_enable_lsi(void);
void RCC_get_lsi_frequency(unsigned int* lsi_frequency_hz);
unsigned char RCC_enable_lse(void);

#endif /* __RCC_H__ */
