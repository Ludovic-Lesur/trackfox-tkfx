/*
 * wifi.c
 *
 *  Created on: 31 mar. 2026
 *      Author: Ludo
 */

#include "wifi.h"

#include "error_base.h"
#include "exti.h"
#include "gpio.h"
#include "lr11xx.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "pwr.h"
#include "rtc.h"
#include "types.h"

#ifdef HW2_0

/*** WIFI local macros ***/

#define WIFI_TCXO_TIMEOUT_MS    10
#define WIFI_TIMEOUT_SECONDS    20

/*** WIFI local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint8_t dio_irq_flag;
    volatile uint8_t dio_irq_enable;
} WIFI_context_t;

/*** WIFI local global variables ***/

static WIFI_context_t wifi_ctx;

/*** WIFI local functions ***/

/*******************************************************************/
static void _WIFI_lr11xx_dio_irq_callback(void) {
    // Set flag if IRQ is enabled.
    wifi_ctx.dio_irq_flag = wifi_ctx.dio_irq_enable;
}

/*******************************************************************/
static WIFI_status_t _WIFI_enable_lr11xx_dio_irq(uint32_t irq_mask) {
    // Local variables.
    WIFI_status_t status = WIFI_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    // Configure interrupt on LR11XX side.
    lr11xx_status = LR11XX_set_dio_irq_mask(irq_mask, 0);
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    // Configure interrupt on MCU side.
    EXTI_configure_gpio(&GPIO_LR1110_DIO9, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, &_WIFI_lr11xx_dio_irq_callback, NVIC_PRIORITY_WIFI_IRQ_GPIO);
    EXTI_clear_gpio_flag(&GPIO_LR1110_DIO9);
    // Enable interrupt.
    wifi_ctx.dio_irq_enable = 1;
    EXTI_enable_gpio_interrupt(&GPIO_LR1110_DIO9);
errors:
    return status;
}

/*******************************************************************/
static void _WIFI_disable_lr11xx_dio_irq(void) {
    // Local variables.
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    // Configure interrupt on LR11XX side.
    lr11xx_status = LR11XX_set_dio_irq_mask(0, 0);
    LR11XX_stack_error(ERROR_BASE_WIFI + WIFI_ERROR_BASE_LR11XX);
    // Disable GPIO interrupt.
    wifi_ctx.dio_irq_enable = 0;
    EXTI_disable_gpio_interrupt(&GPIO_LR1110_DIO9);
    EXTI_release_gpio(&GPIO_LR1110_DIO9, GPIO_MODE_INPUT);
}

/*** WIFI functions ***/

/*******************************************************************/
WIFI_status_t WIFI_init(void) {
    // Local variables.
    WIFI_status_t status = WIFI_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    uint16_t op_error = 0;
    // Init LR11xx.
    lr11xx_status = LR11XX_init();
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    // Init radio.
    lr11xx_status = LR11XX_reset(0);
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    lr11xx_status = LR11XX_set_mode(LR11XX_MODE_STANDBY_RC);
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    lr11xx_status = LR11XX_set_regulation_mode(LR11XX_REGULATION_MODE_DCDC);
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    lr11xx_status = LR11XX_set_oscillator(LR11XX_OSCILLATOR_TCXO, LR11XX_TCXO_VOLTAGE_1V6, WIFI_TCXO_TIMEOUT_MS, LR11XX_LF_CLOCK_INTERNAL_RC);
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    lr11xx_status = LR11XX_clear_errors();
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    lr11xx_status = LR11XX_set_mode(LR11XX_MODE_STANDBY_XOSC);
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    lr11xx_status = LR11XX_get_errors(&op_error);
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    if (op_error != 0) {
        status = WIFI_ERROR_RADIO_INITIALIZATION;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
WIFI_status_t WIFI_de_init(void) {
    // Local variables.
    WIFI_status_t status = WIFI_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    // Init LR11xx.
    lr11xx_status = LR11XX_de_init();
    LR11XX_stack_error(ERROR_BASE_WIFI + WIFI_ERROR_BASE_LR11XX);
    return status;
}

/*******************************************************************/
WIFI_status_t WIFI_scan(WIFI_scan_results_t* wifi_scan_results) {
    // Local variables.
    WIFI_status_t status = WIFI_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    LR11XX_wifi_scan_parameters_t scan_params;
    uint32_t uptime = RTC_get_uptime_seconds();
    uint32_t start_time = uptime;
    // Check parameter.
    if (wifi_scan_results == NULL) {
        status = WIFI_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Reset results.
    wifi_scan_results->number_of_access_points_detected = 0;
    wifi_scan_results->number_of_access_points_written = 0;
    // Scan parameters.
    scan_params.signal_type = LR11XX_WIFI_SIGNAL_TYPE_ALL;
    scan_params.channel_mask = 0x3FFF;
    scan_params.acquisition_mode = LR11XX_WIFI_ACQUISITION_MODE_BEACON_PACKET;
    scan_params.mac_address_list_size = (wifi_scan_results->access_point_list_size);
    scan_params.number_of_scans_per_channel = 5;
    scan_params.single_scan_timeout_ms = 150;
    // Enable GPIO interrupt.
    status = _WIFI_enable_lr11xx_dio_irq(0b1 << LR11XX_IRQ_INDEX_WIFI_DONE);
    if (status != WIFI_SUCCESS) goto errors;
    // Clear flag.
    wifi_ctx.dio_irq_flag = 0;
    // Perform scan.
    lr11xx_status = LR11XX_wifi_scan(&scan_params);
    LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
    // Wait for scan completion.
    while (uptime < (start_time + WIFI_TIMEOUT_SECONDS)) {
        // Ensure RTC is running.
        if (RTC_get_uptime_seconds() > uptime) {
            // Update time and reload watchdog.
            uptime = RTC_get_uptime_seconds();
            IWDG_reload();
        }
        // Enter sleep mode.
        PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STOP);
        // Check interrupt.
        if (wifi_ctx.dio_irq_flag != 0) {
            // Read results.
            lr11xx_status = LR11XX_wifi_read(wifi_scan_results);
            LR11XX_exit_error(WIFI_ERROR_BASE_LR11XX);
            // Exit loop.
            break;
        }
    }
errors:
    _WIFI_disable_lr11xx_dio_irq();
    return status;
}

#endif /* HW2_0 */
