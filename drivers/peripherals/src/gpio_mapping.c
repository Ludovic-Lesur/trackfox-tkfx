/*
 * gpio_mapping.c
 *
 *  Created on: 12 apr. 2024
 *      Author: Ludo
 */

#include "gpio_mapping.h"

#include "gpio.h"
#include "gpio_reg.h"
#include "i2c.h"
#include "lpuart.h"
#include "spi.h"
#include "usart.h"

/*** GPIO MAPPING local global variables ***/

// Analog inputs.
static const GPIO_pin_t GPIO_ADC_LM4040_OUT = { GPIOB, 1, 0, 0 };
static const GPIO_pin_t GPIO_ADC_VSRC_MEASURE = { GPIOA, 0, 6, 0 };
static const GPIO_pin_t GPIO_ADC_VSTR_MEASURE = { GPIOA, 0, 7, 0 };
// Analog inputs list.
static const GPIO_pin_t* GPIO_ADC_PINS_LIST[GPIO_ADC_CHANNEL_LAST] = { &GPIO_ADC_LM4040_OUT, &GPIO_ADC_VSRC_MEASURE, &GPIO_ADC_VSTR_MEASURE };
// I2C1.
static const GPIO_pin_t GPIO_I2C1_SCL = (GPIO_pin_t) {GPIOB, 1, 6, 1};
static const GPIO_pin_t GPIO_I2C1_SDA = (GPIO_pin_t) {GPIOB, 1, 7, 1};
// SPI1.
static const GPIO_pin_t GPIO_SPI1_SCK = (GPIO_pin_t) {GPIOB, 1, 3, 0};
static const GPIO_pin_t GPIO_SPI1_MISO = (GPIO_pin_t) {GPIOB, 1, 4, 0};
static const GPIO_pin_t GPIO_SPI1_MOSI = (GPIO_pin_t) {GPIOB, 1, 5, 0};
// LPUART1.
static const GPIO_pin_t GPIO_LPUART1_TX = (GPIO_pin_t) {GPIOA, 0, 2, 6};
static const GPIO_pin_t GPIO_LPUART1_RX = (GPIO_pin_t) {GPIOA, 0, 3, 6};
// USART2.
static const GPIO_pin_t GPIO_USART2_TX = (GPIO_pin_t) {GPIOA, 0, 9, 4};
static const GPIO_pin_t GPIO_USART2_RX = (GPIO_pin_t) {GPIOA, 0, 10, 4};

/*** GPIO MAPPING global variables ***/

// Analog inputs.
const GPIO_pin_t GPIO_ADC_POWER_ENABLE = (GPIO_pin_t) {GPIOB, 1, 1, 0};
const ADC_gpio_t GPIO_ADC_GPIO = { (const GPIO_pin_t**) &GPIO_ADC_PINS_LIST, GPIO_ADC_CHANNEL_LAST };
// Accelerometer.
#ifdef HW1_0
const GPIO_pin_t GPIO_ACCELERO_IRQ = (GPIO_pin_t) {GPIOA, 0, 1, 0};
#endif
#ifdef HW1_1
const GPIO_pin_t GPIO_ACCELERO_IRQ = (GPIO_pin_t) {GPIOA, 0, 0, 0};
#endif
// GPS.
const GPIO_pin_t GPIO_GPS_POWER_ENABLE = (GPIO_pin_t) {GPIOA, 0, 5, 0};
#ifdef HW1_1
const GPIO_pin_t GPIO_GPS_VBCKP = (GPIO_pin_t) {GPIOA, 0, 1, 0};
#endif
const LPUART_gpio_t GPIO_GPS_LPUART = { &GPIO_LPUART1_TX, &GPIO_LPUART1_RX };
// Radio power control.
const GPIO_pin_t GPIO_RF_POWER_ENABLE = (GPIO_pin_t) {GPIOB, 1, 2, 0};
const GPIO_pin_t GPIO_TCXO_POWER_ENABLE = (GPIO_pin_t) {GPIOA, 0, 8, 0};
// S2LP.
const GPIO_pin_t GPIO_S2LP_CS = (GPIO_pin_t) {GPIOA, 0, 15, 0};
const SPI_gpio_t GPIO_S2LP_SPI = { &GPIO_SPI1_SCK, &GPIO_SPI1_MOSI, &GPIO_SPI1_MISO };
#ifdef HW1_0
const GPIO_pin_t GPIO_S2LP_GPIO3 = (GPIO_pin_t) {GPIOA, 0, 11, 0};
#endif
#ifdef HW1_1
const GPIO_pin_t GPIO_S2LP_SDN = (GPIO_pin_t) {GPIOA, 0, 11, 0};
#endif
const GPIO_pin_t GPIO_S2LP_GPIO0 = (GPIO_pin_t) {GPIOA, 0, 12, 0};
// Sensors.
const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE = (GPIO_pin_t) {GPIOB, 1, 8, 0};
const I2C_gpio_t GPIO_SENSORS_I2C = { &GPIO_I2C1_SCL, &GPIO_I2C1_SDA };
// AT interface.
const USART_gpio_t GPIO_AT_USART = { &GPIO_USART2_TX, &GPIO_USART2_RX };
// Test point.
const GPIO_pin_t GPIO_TP2 = (GPIO_pin_t) {GPIOA, 0, 4, 0};
