/*
 * mapping.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef MAPPING_H
#define MAPPING_H

#include "gpio.h"
#include "gpio_reg.h"

#ifdef HW1_0
// Accelerometer.
static const GPIO GPIO_ACCELERO_IRQ =			(GPIO) {GPIOA, 0, 1, 0};
// LPUART1 (GPS).
static const GPIO GPIO_LPUART1_TX =				(GPIO) {GPIOA, 0, 2, 6};
static const GPIO GPIO_LPUART1_RX =				(GPIO) {GPIOA, 0, 3, 6};
// GPS power control.
static const GPIO GPIO_GPS_POWER_ENABLE =		(GPIO) {GPIOA, 0, 5, 0};
// ADC inputs.
static const GPIO GPIO_ADC1_IN6 =				(GPIO) {GPIOA, 0, 6, 0};
static const GPIO GPIO_ADC1_IN7 =				(GPIO) {GPIOA, 0, 7, 0};
static const GPIO GPIO_ADC1_IN8 =				(GPIO) {GPIOB, 1, 0, 0};
// TCXO power control.
static const GPIO GPIO_TCXO_POWER_ENABLE =		(GPIO) {GPIOA, 0, 8, 0};
// USART2 (AT interface).
static const GPIO GPIO_USART2_TX =				(GPIO) {GPIOA, 0, 9, 4};
static const GPIO GPIO_USART2_RX =				(GPIO) {GPIOA, 0, 10, 4};
// S2LP.
static const GPIO GPIO_S2LP_GPIO3 =				(GPIO) {GPIOA, 0, 11, 0};
static const GPIO GPIO_S2LP_GPIO0 =				(GPIO) {GPIOA, 0, 12, 0};
// Programming.
static const GPIO GPIO_SWDIO =					(GPIO) {GPIOA, 0, 13, 0};
static const GPIO GPIO_SWCLK =					(GPIO) {GPIOA, 0, 14, 0};
// SPI1 (S2LP).
static const GPIO GPIO_SPI1_SCK = 				(GPIO) {GPIOB, 1, 3, 0};
static const GPIO GPIO_SPI1_MISO = 				(GPIO) {GPIOB, 1, 4, 0};
static const GPIO GPIO_SPI1_MOSI = 				(GPIO) {GPIOB, 1, 5, 0};
//static const GPIO GPIO_S2LP_CS = 				(GPIO) {GPIOA, 0, 15, 0};
static const GPIO GPIO_S2LP_CS = 				(GPIO) {GPIOB, 1, 1, 0}; // Test with Nucleo.
// ADC power enable.
static const GPIO GPIO_ADC_POWER_ENABLE =		(GPIO) {GPIOB, 1, 1, 0};
// Test point.
static const GPIO GPIO_TP2 =					(GPIO) {GPIOA, 0, 4, 0};
// RF power enable.
//static const GPIO GPIO_RF_POWER_ENABLE =		(GPIO) {GPIOB, 1, 2, 0};
static const GPIO GPIO_RF_POWER_ENABLE =		(GPIO) {GPIOA, 0, 11, 0}; // Test with Nucleo.
// I2C (sensors).
static const GPIO GPIO_I2C1_SCL	=				(GPIO) {GPIOB, 1, 6, 1};
static const GPIO GPIO_I2C1_SDA	=				(GPIO) {GPIOB, 1, 7, 1};
// Sensors power control.
static const GPIO GPIO_SENSORS_POWER_ENABLE =	(GPIO) {GPIOB, 1, 8, 0};
#endif

#endif /* MAPPING_H */
