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

// Accelerometer.
#ifdef HW1_0
static const GPIO_pin_t GPIO_ACCELERO_IRQ =			(GPIO_pin_t) {GPIOA, 0, 1, 0};
#endif
#ifdef HW1_1
static const GPIO_pin_t GPIO_ACCELERO_IRQ =			(GPIO_pin_t) {GPIOA, 0, 0, 0};
#endif
// LPUART1 (GPS).
static const GPIO_pin_t GPIO_LPUART1_TX =			(GPIO_pin_t) {GPIOA, 0, 2, 6};
static const GPIO_pin_t GPIO_LPUART1_RX =			(GPIO_pin_t) {GPIOA, 0, 3, 6};
// GPS power control and backup.
static const GPIO_pin_t GPIO_GPS_POWER_ENABLE =		(GPIO_pin_t) {GPIOA, 0, 5, 0};
#ifdef HW1_1
static const GPIO_pin_t GPIO_GPS_VBCKP =			(GPIO_pin_t) {GPIOA, 0, 1, 0};
#endif
// ADC inputs.
static const GPIO_pin_t GPIO_ADC1_IN6 =				(GPIO_pin_t) {GPIOA, 0, 6, 0};
static const GPIO_pin_t GPIO_ADC1_IN7 =				(GPIO_pin_t) {GPIOA, 0, 7, 0};
static const GPIO_pin_t GPIO_ADC1_IN8 =				(GPIO_pin_t) {GPIOB, 1, 0, 0};
// TCXO power control.
static const GPIO_pin_t GPIO_TCXO_POWER_ENABLE =	(GPIO_pin_t) {GPIOA, 0, 8, 0};
// USART2 (AT interface).
static const GPIO_pin_t GPIO_USART2_TX =			(GPIO_pin_t) {GPIOA, 0, 9, 4};
static const GPIO_pin_t GPIO_USART2_RX =			(GPIO_pin_t) {GPIOA, 0, 10, 4};
// S2LP.
#ifdef HW1_0
static const GPIO_pin_t GPIO_S2LP_GPIO3 =			(GPIO_pin_t) {GPIOA, 0, 11, 0};
#endif
#ifdef HW1_1
static const GPIO_pin_t GPIO_S2LP_SDN =				(GPIO_pin_t) {GPIOA, 0, 11, 0};
#endif
static const GPIO_pin_t GPIO_S2LP_GPIO0 =			(GPIO_pin_t) {GPIOA, 0, 12, 0};
// Programming.
static const GPIO_pin_t GPIO_SWDIO =				(GPIO_pin_t) {GPIOA, 0, 13, 0};
static const GPIO_pin_t GPIO_SWCLK =				(GPIO_pin_t) {GPIOA, 0, 14, 0};
// SPI1 (S2LP).
static const GPIO_pin_t GPIO_SPI1_SCK = 			(GPIO_pin_t) {GPIOB, 1, 3, 0};
static const GPIO_pin_t GPIO_SPI1_MISO = 			(GPIO_pin_t) {GPIOB, 1, 4, 0};
static const GPIO_pin_t GPIO_SPI1_MOSI = 			(GPIO_pin_t) {GPIOB, 1, 5, 0};
static const GPIO_pin_t GPIO_S2LP_CS = 				(GPIO_pin_t) {GPIOA, 0, 15, 0};
// ADC power enable.
static const GPIO_pin_t GPIO_ADC_POWER_ENABLE =		(GPIO_pin_t) {GPIOB, 1, 1, 0};
// Test point.
static const GPIO_pin_t GPIO_TP2 =					(GPIO_pin_t) {GPIOA, 0, 4, 0};
// RF power enable.
static const GPIO_pin_t GPIO_RF_POWER_ENABLE =		(GPIO_pin_t) {GPIOB, 1, 2, 0};
// I2C (sensors).
static const GPIO_pin_t GPIO_I2C1_SCL	=			(GPIO_pin_t) {GPIOB, 1, 6, 1};
static const GPIO_pin_t GPIO_I2C1_SDA	=			(GPIO_pin_t) {GPIOB, 1, 7, 1};
// Sensors power control.
static const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE =	(GPIO_pin_t) {GPIOB, 1, 8, 0};

#endif /* MAPPING_H */
