/*
 * mcu_mapping.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __MCU_MAPPING_H__
#define __MCU_MAPPING_H__

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "lpuart.h"
#include "spi.h"
#include "usart.h"

/*** MCU MAPPING macros ***/

#define ADC_CHANNEL_VSRC        ADC_CHANNEL_IN6
#define ADC_CHANNEL_VSTR        ADC_CHANNEL_IN7
#define ADC_CHANNEL_LM4040      ADC_CHANNEL_IN8

#define I2C_INSTANCE_SENSORS    I2C_INSTANCE_I2C1

#define SPI_INSTANCE_RADIO      SPI_INSTANCE_SPI1

#define TIM_INSTANCE_MCU_API    TIM_INSTANCE_TIM2

#define USART_INSTANCE_GPS      USART_INSTANCE_USART2
#define USART_INSTANCE_AT       USART_INSTANCE_USART1

/*** MCU MAPPING structures ***/

/*!******************************************************************
 * \enum ADC_channel_index_t
 * \brief ADC channels index.
 *******************************************************************/
typedef enum {
    ADC_CHANNEL_INDEX_LM4040_OUT = 0,
    ADC_CHANNEL_INDEX_VSRC_MEASURE,
    ADC_CHANNEL_INDEX_VCAP_MEASURE,
    ADC_CHANNEL_INDEX_LAST
} ADC_channel_index_t;

/*** MCU MAPPING global variables ***/

// Analog inputs.
extern const GPIO_pin_t GPIO_ADC_POWER_ENABLE;
extern const ADC_gpio_t ADC_GPIO;
// Accelerometer.
extern const GPIO_pin_t GPIO_ACCELERO_IRQ;
// GPS.
extern const GPIO_pin_t GPIO_GPS_POWER_ENABLE;
#ifdef HW1_1
extern const GPIO_pin_t GPIO_GPS_VBCKP;
#endif
extern const USART_gpio_t USART_GPIO_GPS;
// Radio power control.
extern const GPIO_pin_t GPIO_RF_POWER_ENABLE;
extern const GPIO_pin_t GPIO_TCXO_POWER_ENABLE;
// S2LP.
extern const SPI_gpio_t SPI_GPIO_S2LP;
extern const GPIO_pin_t GPIO_S2LP_CS;
extern const GPIO_pin_t GPIO_S2LP_GPIO0;
#ifdef HW1_0
extern const GPIO_pin_t GPIO_S2LP_GPIO3;
#endif
#ifdef HW1_1
extern const GPIO_pin_t GPIO_S2LP_SDN;
#endif
// Sensors.
extern const GPIO_pin_t GPIO_SENSORS_POWER_ENABLE;
extern const I2C_gpio_t I2C_GPIO_SENSORS;
// AT interface.
extern const USART_gpio_t USART_GPIO_AT;
// Test point.
extern const GPIO_pin_t GPIO_TP2;

#endif /* __MCU_MAPPING_H__ */
