/*
 * led.c
 *
 *  Created on: 20 mar. 2026
 *      Author: Ludo
 */

#include "led.h"

#include "error.h"
#include "error_base.h"
#include "gpio.h"
#include "mcu_mapping.h"
#include "types.h"

#ifdef HW2_0

/*** LED functions ***/

/*******************************************************************/
LED_status_t LED_init(void) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    // Configure LED pins.
    GPIO_configure(&GPIO_LED_RED, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_LED_GREEN, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&GPIO_LED_BLUE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    return status;
}

/*******************************************************************/
LED_status_t LED_de_init(void) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    // Set all pins to output low.
    status = LED_set_color(LED_COLOR_OFF);
    if (status != LED_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
LED_status_t LED_set_color(LED_color_t led_color) {
    // Local variables.
    LED_status_t status = LED_SUCCESS;
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    // Check color.
    switch (led_color) {
    case LED_COLOR_OFF:
        // Nothing to do.
        break;
    case LED_COLOR_RED:
        red = 1;
        break;
    case LED_COLOR_GREEN:
        green = 1;
        break;
    case LED_COLOR_YELLOW:
        red = 1;
        green = 1;
        break;
    case LED_COLOR_BLUE:
        blue = 1;
        break;
    case LED_COLOR_MAGENTA:
        red = 1;
        blue = 1;
        break;
    case LED_COLOR_CYAN:
        green = 1;
        blue = 1;
        break;
    case LED_COLOR_WHITE:
        red = 1;
        green = 1;
        blue = 1;
        break;
    default:
        status = LED_ERROR_COLOR;
        goto errors;
    }
    GPIO_write(&GPIO_LED_RED, red);
    GPIO_write(&GPIO_LED_GREEN, green);
    GPIO_write(&GPIO_LED_BLUE, blue);
errors:
    return status;
}

#endif /* HW2_0 */
