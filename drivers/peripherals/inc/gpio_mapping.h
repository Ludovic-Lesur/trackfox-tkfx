/*
 * gpio_mapping.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __GPIO_MAPPING_H__
#define __GPIO_MAPPING_H__

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "lpuart.h"
#include "spi.h"
#include "usart.h"

/*** GPIO MAPPING global variables ***/

// Analog inputs.
extern const GPIO_pin_t GPIO_ADC_POWER_ENABLE;
extern const ADC_gpio_t GPIO_ADC;
// Accelerometer.
extern const GPIO_pin_t GPIO_ACCELERO_IRQ;
// GPS.
extern const GPIO_pin_t GPIO_GPS_POWER_ENABLE;
#ifdef HW1_1
extern const GPIO_pin_t GPIO_GPS_VBCKP;
#endif
extern const LPUART_gpio_t GPIO_GPS_LPUART;
// Radio power control.
extern const GPIO_pin_t GPIO_RF_POWER_ENABLE;
extern const GPIO_pin_t GPIO_TCXO_POWER_ENABLE;
// S2LP.
extern const GPIO_pin_t GPIO_S2LP_CS;
extern const SPI_gpio_t GPIO_S2LP_SPI;
extern const GPIO_pin_t GPIO_S2LP_GPIO0;
#ifdef HW1_0
extern const GPIO_pin_t GPIO_S2LP_GPIO3;
#endif
#ifdef HW1_1
extern const GPIO_pin_t GPIO_S2LP_SDN;
#endif
// Sensors.
extern const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE;
extern const I2C_gpio_t GPIO_SENSORS_I2C;
// AT interface.
extern const USART_gpio_t GPIO_AT_USART;
// Test point.
extern const GPIO_pin_t GPIO_TP2;

#endif /* __GPIO_MAPPING_H__ */
