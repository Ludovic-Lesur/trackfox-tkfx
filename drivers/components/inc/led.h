/*
 * led.h
 *
 *  Created on: 20 mar. 2026
 *      Author: Ludo
 */

#ifndef __LED_H__
#define __LED_H__

#include "error.h"
#include "types.h"

/*** LED structures ***/

/*!******************************************************************
 * \enum LED_status_t
 * \brief LED driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    LED_SUCCESS,
    LED_ERROR_COLOR,
    // Last base value.
    LED_ERROR_BASE_LAST = ERROR_BASE_STEP
} LED_status_t;

#ifdef HW2_0

/*!******************************************************************
 * \enum LED_color_t
 * \brief LED colors list.
 *******************************************************************/
typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_YELLOW,
    LED_COLOR_BLUE,
    LED_COLOR_MAGENTA,
    LED_COLOR_CYAN,
    LED_COLOR_WHITE,
    LED_COLOR_LAST
} LED_color_t;

/*** LED functions ***/

/*!******************************************************************
 * \fn LED_status_t LED_init(void)
 * \brief Init LED driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_init(void);

/*!******************************************************************
 * \fn LED_status_t LED_de_init(void)
 * \brief Release LED driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_de_init(void);

/*!******************************************************************
 * \fn LED_status_t LED_set_color(LED_color_t led_color)
 * \brief Set LED color.
 * \param[in]   led_color: Color to select.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LED_status_t LED_set_color(LED_color_t led_color);

/*******************************************************************/
#define LED_exit_error(base) { ERROR_check_exit(led_status, LED_SUCCESS, base) }

/*******************************************************************/
#define LED_stack_error(base) { ERROR_check_stack(led_status, LED_SUCCESS, base) }

/*******************************************************************/
#define LED_stack_exit_error(base, code) { ERROR_check_stack_exit(led_status, LED_SUCCESS, base, code) }

#endif /* HW2_0 */

#endif /* __LED_H__ */
