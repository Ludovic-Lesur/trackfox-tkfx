/*
 * mapping.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __MAPPING_H__
#define __MAPPING_H__

#include "gpio.h"

/*** MAPPING global variables ***/

// Accelerometer.
#ifdef HW1_0
extern const GPIO_pin_t GPIO_ACCELERO_IRQ;
#endif
#ifdef HW1_1
extern const GPIO_pin_t GPIO_ACCELERO_IRQ;
#endif
// LPUART1 (GPS).
extern const GPIO_pin_t GPIO_LPUART1_TX;
extern const GPIO_pin_t GPIO_LPUART1_RX;
// GPS power control and backup.
extern const GPIO_pin_t GPIO_GPS_POWER_ENABLE;
#ifdef HW1_1
extern const GPIO_pin_t GPIO_GPS_VBCKP;
#endif
// ADC inputs.
extern const GPIO_pin_t GPIO_ADC1_IN6;
extern const GPIO_pin_t GPIO_ADC1_IN7;
extern const GPIO_pin_t GPIO_ADC1_IN8;
// TCXO power control.
extern const GPIO_pin_t GPIO_TCXO_POWER_ENABLE;
// USART2 (AT interface).
extern const GPIO_pin_t GPIO_USART2_TX;
extern const GPIO_pin_t GPIO_USART2_RX;
// S2LP.
#ifdef HW1_0
extern const GPIO_pin_t GPIO_S2LP_GPIO3;
#endif
#ifdef HW1_1
extern const GPIO_pin_t GPIO_S2LP_SDN;
#endif
extern const GPIO_pin_t GPIO_S2LP_GPIO0;
// Programming.
extern const GPIO_pin_t GPIO_SWDIO;
extern const GPIO_pin_t GPIO_SWCLK;
// SPI1 (S2LP).
extern const GPIO_pin_t GPIO_SPI1_SCK;
extern const GPIO_pin_t GPIO_SPI1_MISO;
extern const GPIO_pin_t GPIO_SPI1_MOSI;
extern const GPIO_pin_t GPIO_S2LP_CS;
// ADC power enable.
extern const GPIO_pin_t GPIO_ADC_POWER_ENABLE;
// Test point.
extern const GPIO_pin_t GPIO_TP2;
// RF power enable.
extern const GPIO_pin_t GPIO_RF_POWER_ENABLE;
// I2C (sensors).
extern const GPIO_pin_t GPIO_I2C1_SCL;
extern const GPIO_pin_t GPIO_I2C1_SDA;
// Sensors power control.
extern const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE;

#endif /* __MAPPING_H__ */
