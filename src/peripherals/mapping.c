/*
 * mapping.c
 *
 *  Created on: 12 apr. 2024
 *      Author: Ludo
 */

#include "mapping.h"

#include "gpio.h"
#include "gpio_reg.h"

// Accelerometer.
#ifdef HW1_0
const GPIO_pin_t GPIO_ACCELERO_IRQ =			(GPIO_pin_t) {GPIOA, 0, 1, 0};
#endif
#ifdef HW1_1
const GPIO_pin_t GPIO_ACCELERO_IRQ =			(GPIO_pin_t) {GPIOA, 0, 0, 0};
#endif
// LPUART1 (GPS).
const GPIO_pin_t GPIO_LPUART1_TX =				(GPIO_pin_t) {GPIOA, 0, 2, 6};
const GPIO_pin_t GPIO_LPUART1_RX =				(GPIO_pin_t) {GPIOA, 0, 3, 6};
// GPS power control and backup.
const GPIO_pin_t GPIO_GPS_POWER_ENABLE =		(GPIO_pin_t) {GPIOA, 0, 5, 0};
#ifdef HW1_1
const GPIO_pin_t GPIO_GPS_VBCKP =				(GPIO_pin_t) {GPIOA, 0, 1, 0};
#endif
// ADC inputs.
const GPIO_pin_t GPIO_ADC1_IN6 =				(GPIO_pin_t) {GPIOA, 0, 6, 0};
const GPIO_pin_t GPIO_ADC1_IN7 =				(GPIO_pin_t) {GPIOA, 0, 7, 0};
const GPIO_pin_t GPIO_ADC1_IN8 =				(GPIO_pin_t) {GPIOB, 1, 0, 0};
// TCXO power control.
const GPIO_pin_t GPIO_TCXO_POWER_ENABLE =		(GPIO_pin_t) {GPIOA, 0, 8, 0};
// USART2 (AT interface).
const GPIO_pin_t GPIO_USART2_TX =				(GPIO_pin_t) {GPIOA, 0, 9, 4};
const GPIO_pin_t GPIO_USART2_RX =				(GPIO_pin_t) {GPIOA, 0, 10, 4};
// S2LP.
#ifdef HW1_0
const GPIO_pin_t GPIO_S2LP_GPIO3 =				(GPIO_pin_t) {GPIOA, 0, 11, 0};
#endif
#ifdef HW1_1
const GPIO_pin_t GPIO_S2LP_SDN =				(GPIO_pin_t) {GPIOA, 0, 11, 0};
#endif
const GPIO_pin_t GPIO_S2LP_GPIO0 =				(GPIO_pin_t) {GPIOA, 0, 12, 0};
// Programming.
const GPIO_pin_t GPIO_SWDIO =					(GPIO_pin_t) {GPIOA, 0, 13, 0};
const GPIO_pin_t GPIO_SWCLK =					(GPIO_pin_t) {GPIOA, 0, 14, 0};
// SPI1 (S2LP).
const GPIO_pin_t GPIO_SPI1_SCK = 				(GPIO_pin_t) {GPIOB, 1, 3, 0};
const GPIO_pin_t GPIO_SPI1_MISO = 				(GPIO_pin_t) {GPIOB, 1, 4, 0};
const GPIO_pin_t GPIO_SPI1_MOSI = 				(GPIO_pin_t) {GPIOB, 1, 5, 0};
const GPIO_pin_t GPIO_S2LP_CS = 				(GPIO_pin_t) {GPIOA, 0, 15, 0};
// ADC power enable.
const GPIO_pin_t GPIO_ADC_POWER_ENABLE =		(GPIO_pin_t) {GPIOB, 1, 1, 0};
// Test point.
const GPIO_pin_t GPIO_TP2 =						(GPIO_pin_t) {GPIOA, 0, 4, 0};
// RF power enable.
const GPIO_pin_t GPIO_RF_POWER_ENABLE =			(GPIO_pin_t) {GPIOB, 1, 2, 0};
// I2C (sensors).
const GPIO_pin_t GPIO_I2C1_SCL	=				(GPIO_pin_t) {GPIOB, 1, 6, 1};
const GPIO_pin_t GPIO_I2C1_SDA	=				(GPIO_pin_t) {GPIOB, 1, 7, 1};
// Sensors power control.
const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE =	(GPIO_pin_t) {GPIOB, 1, 8, 0};
