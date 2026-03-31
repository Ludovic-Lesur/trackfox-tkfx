/*!*****************************************************************
 * \file    rf_api.c
 * \brief   Radio drivers.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "manuf/rf_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#include "error.h"
#include "error_base.h"
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "led.h"
#include "lr11xx.h"
#include "manuf/mcu_api.h"
#include "mcu_mapping.h"
#include "nvic_priority.h"
#include "power.h"
#include "pwr.h"
#include "rfe.h"
#include "tim.h"
#include "types.h"

#ifdef HW2_0

/*** RF API local macros ***/

#define RF_API_FREQUENCY_MIN_MHZ                863
#define RF_API_FREQUENCY_MAX_MHZ                870

#define RF_API_TCXO_TIMEOUT_MS                  10

#define RF_API_RADIO_SEND_START_LATENCY_BITS    2
#define RF_API_RADIO_SEND_STOP_LATENCY_BITS     9

#define RF_API_TIMER_CHANNEL                    TIM_CHANNEL_4
#define RF_API_TX_TIMEOUT_MS                    3000

#ifdef SIGFOX_EP_BIDIRECTIONAL
#define RF_API_DL_PR_SIZE_BITS                  16
#endif

/*** RF API local structures ***/

/*******************************************************************/
typedef enum {
    // Driver errors.
    RF_API_ERROR_NULL_PARAMETER = (RF_API_SUCCESS + 1),
    RF_API_ERROR_BUFFER_SIZE,
    RF_API_ERROR_STATE,
    RF_API_ERROR_MODULATION,
    RF_API_ERROR_BIT_RATE,
    RF_API_ERROR_MODE,
    RF_API_ERROR_LR11XX_STATUS,
    RF_API_ERROR_TX_TIMEOUT,
    RF_API_ERROR_LATENCY_TYPE,
    // Low level drivers errors.
    RF_API_ERROR_DRIVER_MCU_API,
    RF_API_ERROR_DRIVER_LR11XX,
    RF_API_ERROR_DRIVER_RFE,
    RF_API_ERROR_DRIVER_TIM,
    RF_API_ERROR_DRIVER_LED
} RF_API_custom_status_t;

/*******************************************************************/
typedef union {
    sfx_u8 all;
    struct {
        unsigned dio_irq_enable :1;
        unsigned dio_irq_flag :1;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} RF_API_flags_t;

/*******************************************************************/
typedef struct {
    volatile RF_API_flags_t flags;
    // TX.
    sfx_u16 bit_rate_bps;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // RX.
    sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm;
#endif
} RF_API_context_t;

/*** RF API local global variables ***/

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
static sfx_u32 RF_API_LATENCY_MS[RF_API_LATENCY_LAST] = {
    POWER_ON_DELAY_MS_TCXO, // Wake-up.
    (POWER_ON_DELAY_MS_RADIO + LR11XX_EXIT_RESET_DELAY_MS + LR11XX_CALIBRATION_DELAY_MS + (RF_API_TCXO_TIMEOUT_MS << 1) + 1), // TX init.
    0, // Send start (depends on bit rate and will be computed during init function).
    0, // Send stop (depends on bit rate and will be computed during init function).
    0, // TX de-init (100us).
    0, // Sleep.
#ifdef SIGFOX_EP_BIDIRECTIONAL
    (POWER_ON_DELAY_MS_RADIO + LR11XX_EXIT_RESET_DELAY_MS + LR11XX_CALIBRATION_DELAY_MS + (RF_API_TCXO_TIMEOUT_MS << 1) + 1), // RX init.
    0, // Receive start.
    5, // Receive stop.
    0, // RX de-init (100us).
#endif
};
#endif
static RF_API_context_t rf_api_ctx;

/*** RF API local functions ***/

/*******************************************************************/
static void _RF_API_lr11xx_dio_irq_callback(void) {
    // Set flag if IRQ is enabled.
    rf_api_ctx.flags.dio_irq_flag = rf_api_ctx.flags.dio_irq_enable;
}

/*******************************************************************/
static RF_API_status_t _RF_API_enable_lr11xx_dio_irq(sfx_u32 irq_mask) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    // Configure interrupt on LR11XX side.
    lr11xx_status = LR11XX_set_dio_irq_mask(irq_mask, 0);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Configure interrupt on MCU side.
    EXTI_configure_gpio(&GPIO_LR1110_DIO9, GPIO_PULL_NONE, EXTI_TRIGGER_RISING_EDGE, &_RF_API_lr11xx_dio_irq_callback, NVIC_PRIORITY_SIGFOX_RADIO_IRQ_GPIO);
    EXTI_clear_gpio_flag(&GPIO_LR1110_DIO9);
    // Enable interrupt.
    rf_api_ctx.flags.dio_irq_enable = 1;
    EXTI_enable_gpio_interrupt(&GPIO_LR1110_DIO9);
errors:
    return status;
}

/*******************************************************************/
static void _RF_API_disable_lr11xx_dio_irq(void) {
    // Local variables.
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    // Configure interrupt on LR11XX side.
    lr11xx_status = LR11XX_set_dio_irq_mask(0, 0);
    LR11XX_stack_error(ERROR_BASE_LR1110);
    // Disable GPIO interrupt.
    rf_api_ctx.flags.dio_irq_enable = 0;
    EXTI_disable_gpio_interrupt(&GPIO_LR1110_DIO9);
    EXTI_release_gpio(&GPIO_LR1110_DIO9, GPIO_MODE_INPUT);
}

/*** RF API functions ***/

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t* rf_api_config) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    // Ignore unused parameters.
    UNUSED(rf_api_config);
    // Return.
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t RF_API_close(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t RF_API_process(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t RF_API_wake_up(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    // Turn radio TCXO on.
    POWER_enable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_TCXO, LPTIM_DELAY_MODE_SLEEP);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_sleep(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    // Turn radio TCXO off.
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_TCXO);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t* radio_parameters) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    LR11XX_modulation_parameters_t modulation_parameters;
    LR11XX_pa_t pa = LR11XX_PA_LAST;
    RFE_path_t path = RFE_PATH_LAST;
    sfx_u16 op_error = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_u8 dl_ft[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
    LR11XX_gfsk_packet_parameters_t gfsk_packet_parameters;
    sfx_u8 idx = 0;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (radio_parameters == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Turn radio on.
    POWER_enable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO, LPTIM_DELAY_MODE_SLEEP);
    // Exit reset.
    lr11xx_status = LR11XX_reset(0);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Enter standby RC mode.
    lr11xx_status = LR11XX_set_mode(LR11XX_MODE_STANDBY_RC);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Regulation mode.
    lr11xx_status = LR11XX_set_regulation_mode(LR11XX_REGULATION_MODE_DCDC);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Oscillator.
    lr11xx_status = LR11XX_set_oscillator(LR11XX_OSCILLATOR_TCXO, LR11XX_TCXO_VOLTAGE_1V6, RF_API_TCXO_TIMEOUT_MS, LR11XX_LF_CLOCK_INTERNAL_RC);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Clear device errors.
    lr11xx_status = LR11XX_clear_errors();
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Calibration.
    lr11xx_status = LR11XX_calibrate(RF_API_FREQUENCY_MIN_MHZ, RF_API_FREQUENCY_MAX_MHZ);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Enter standby XOSC mode.
    lr11xx_status = LR11XX_set_mode(LR11XX_MODE_STANDBY_XOSC);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Frequency.
    lr11xx_status = LR11XX_set_rf_frequency(radio_parameters->frequency_hz);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Update context.
    rf_api_ctx.bit_rate_bps = (radio_parameters->bit_rate_bps);
    // Common modulation parameters.
#ifdef SIGFOX_EP_BIDIRECTIONAL
    modulation_parameters.fsk_deviation_hz = (radio_parameters->deviation_hz);
#else
    modulation_parameters.fsk_deviation_hz = 0;
#endif
    modulation_parameters.rx_bandwidth = LR11XX_RXBW_4800HZ;
    // Modulation parameters.
    switch (radio_parameters->modulation) {
    case RF_API_MODULATION_NONE:
        modulation_parameters.bit_rate_bps = 100;
        modulation_parameters.modulation = LR11XX_MODULATION_GFSK;
        modulation_parameters.modulation_shaping = LR11XX_MODULATION_SHAPING_NONE;
        break;
    case RF_API_MODULATION_DBPSK:
        modulation_parameters.bit_rate_bps = (radio_parameters->bit_rate_bps);
        modulation_parameters.modulation = LR11XX_MODULATION_BPSK;
        modulation_parameters.modulation_shaping = LR11XX_MODULATION_SHAPING_DBPSK;

        break;
    case RF_API_MODULATION_GFSK:
        modulation_parameters.bit_rate_bps = (radio_parameters->bit_rate_bps);
        modulation_parameters.modulation = LR11XX_MODULATION_GFSK;
        modulation_parameters.modulation_shaping = LR11XX_MODULATION_SHAPING_GAUSSIAN_BT_1;
        break;
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_MODULATION);
        break;
    }
    // Set modulation scheme.
    lr11xx_status = LR11XX_set_modulation(&modulation_parameters);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Configure specific registers.
    switch (radio_parameters->rf_mode) {
    case RF_API_MODE_TX:
        // Select pin.
        if ((radio_parameters->tx_power_dbm_eirp) > LR11XX_RF_OUTPUT_POWER_LP_PA_DBM_MAX) {
            // Use HP PA.
            pa = LR11XX_PA_HIGH_POWER;
            path = RFE_PATH_TX_HIGH_POWER;
        }
        else {
            // Use LP PA.
            pa = LR11XX_PA_LOW_POWER;
            path = RFE_PATH_TX_LOW_POWER;
        }
        // Set output power.
        lr11xx_status = LR11XX_set_rf_output_power(pa, (radio_parameters->tx_power_dbm_eirp), LR11XX_PA_RAMP_TIME_208U);
        LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
        // Start latency = ramp-up.
        RF_API_LATENCY_MS[RF_API_LATENCY_SEND_START] = (((RF_API_RADIO_SEND_START_LATENCY_BITS * 1000) / ((sfx_u32) (radio_parameters->bit_rate_bps))) + RF_API_TCXO_TIMEOUT_MS);
        // Stop latency = ramp-down + half of padding bit (since IRQ is raised at the middle of the symbol).
        RF_API_LATENCY_MS[RF_API_LATENCY_SEND_STOP] = (((RF_API_RADIO_SEND_STOP_LATENCY_BITS * 1000) / ((sfx_u32) (radio_parameters->bit_rate_bps))) + RF_API_TCXO_TIMEOUT_MS);
#endif
        // Switch to TX.
        rfe_status = RFE_set_path(path);
        RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case RF_API_MODE_RX:
        // LNA mode.
        lr11xx_status = LR11XX_set_lna_mode(LR11XX_LNA_MODE_BOOST);
        LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
        // Downlink packet structure.
        gfsk_packet_parameters.preamble_length_bits = RF_API_DL_PR_SIZE_BITS;
        gfsk_packet_parameters.preamble_detector_length = LR11XX_PREAMBLE_DETECTOR_LENGTH_16BITS;
        for (idx = 0; idx < LR11XX_SYNC_WORD_SIZE_BYTES_MAX; idx++) {
            gfsk_packet_parameters.sync_word[idx] = (idx < SIGFOX_DL_FT_SIZE_BYTES) ? dl_ft[idx] : 0x00;
        }
        gfsk_packet_parameters.sync_word_length_bits = (SIGFOX_DL_FT_SIZE_BYTES << 3);
        gfsk_packet_parameters.payload_length_bytes = SIGFOX_DL_PHY_CONTENT_SIZE_BYTES;
        lr11xx_status = LR11XX_set_gfsk_packet(&gfsk_packet_parameters);
        LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
        // Clear FIFO.
        lr11xx_status = LR11XX_clear_fifo();
        LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
        // Switch to RX.
        rfe_status = RFE_set_path(RFE_PATH_RX_LNA);
        RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_MODE);
        break;
    }
    // Read device errors at the end of initialization.
    lr11xx_status = LR11XX_get_errors(&op_error);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Exit if there any error in device status.
    if (op_error != 0) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_LR11XX_STATUS);
    }
    // Turn LED on.
    led_status = LED_set_color(LED_COLOR_BLUE);
    LED_stack_exit_error(ERROR_BASE_LED, (RF_API_status_t) RF_API_ERROR_DRIVER_LED);
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_de_init(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    LED_status_t led_status = LED_SUCCESS;
    // Disable front-end.
    rfe_status = RFE_set_path(RFE_PATH_NONE);
    // Check status.
    if (rfe_status != RFE_SUCCESS) {
        RFE_stack_error(ERROR_BASE_RFE);
        status = (RF_API_status_t) RF_API_ERROR_DRIVER_RFE;
    }
    // Turn transceiver off.
    lr11xx_status = LR11XX_reset(1);
    // Check status.
    if (lr11xx_status != LR11XX_SUCCESS) {
        LR11XX_stack_error(ERROR_BASE_LR1110);
        status = (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX;
    }
    // Turn LED off.
    led_status = LED_set_color(LED_COLOR_OFF);
    // Check status.
    if (led_status != LED_SUCCESS) {
        LED_stack_error(ERROR_BASE_LED);
        status = (RF_API_status_t) RF_API_ERROR_DRIVER_LED;
    }
    // Turn radio off.
    POWER_disable(POWER_REQUESTER_ID_RF_API, POWER_DOMAIN_RADIO);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t* tx_data) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    LR11XX_bpsk_packet_parameters_t bpsk_packet_parameters;
    sfx_u8 buffer[SIGFOX_UL_BITSTREAM_SIZE_BYTES + 1];
    sfx_u8 differential_size_bytes = 0;
    sfx_u16 differential_size_bits = 0;
    sfx_u8 timer_has_elapsed = 0;
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (tx_data == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Reset flags.
    rf_api_ctx.flags.all = 0;
    // Differential encoding.
    for (idx = 0; idx < (tx_data->bitstream_size_bytes); idx++) {
        buffer[idx] = tx_data->bitstream[idx];
    }
    buffer[tx_data->bitstream_size_bytes] = 0x80;
    lr11xx_status = LR11XX_differential_encoding(buffer, (tx_data->bitstream_size_bytes), buffer, &differential_size_bytes, &differential_size_bits);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Set packet parameters.
    bpsk_packet_parameters.payload_length_bytes = differential_size_bytes;
    bpsk_packet_parameters.payload_length_bits = differential_size_bits;
    // Ramp times.
    switch (rf_api_ctx.bit_rate_bps) {
    case 100:
        bpsk_packet_parameters.ramp_up_delay = LR11XX_RAMP_UP_DELAY_DBPSK_100BPS;
        bpsk_packet_parameters.ramp_down_delay = LR11XX_RAMP_DOWN_DELAY_DBPSK_100BPS;
        break;
    case 600:
        bpsk_packet_parameters.ramp_up_delay = LR11XX_RAMP_UP_DELAY_DBPSK_600BPS;
        bpsk_packet_parameters.ramp_down_delay = LR11XX_RAMP_DOWN_DELAY_DBPSK_600BPS;
        break;
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_BIT_RATE);
    }
    lr11xx_status = LR11XX_set_bpsk_packet(&bpsk_packet_parameters);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Write FIFO.
    lr11xx_status = LR11XX_write_fifo(buffer, differential_size_bytes);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Enable GPIO interrupt.
    status = _RF_API_enable_lr11xx_dio_irq(0b1 << LR11XX_IRQ_INDEX_TX_DONE);
    if (status != RF_API_SUCCESS) goto errors;
    // Start timer (TIM MCH has been initialized by the MCU_API
    tim_status = TIM_MCH_start_channel(TIM_INSTANCE_MCU_API, RF_API_TIMER_CHANNEL, RF_API_TX_TIMEOUT_MS, TIM_WAITING_MODE_ACTIVE);
    TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (RF_API_status_t) RF_API_ERROR_DRIVER_TIM);
    // Start transmission.
    lr11xx_status = LR11XX_set_mode(LR11XX_MODE_TX);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Wait for transmission to complete.
    while (rf_api_ctx.flags.dio_irq_flag == 0) {
        // Enter sleep mode.
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        // Check timer.
        tim_status = TIM_MCH_get_channel_status(TIM_INSTANCE_MCU_API, RF_API_TIMER_CHANNEL, &timer_has_elapsed);
        TIM_stack_exit_error(ERROR_BASE_TIM_MCU_API, (RF_API_status_t) RF_API_ERROR_DRIVER_TIM);
        // Exit if timeout.
        if (timer_has_elapsed != 0) {
            SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_TX_TIMEOUT);
        }
    }
errors:
    // Stop timer and disable GPIO interrupt.
    TIM_MCH_stop_channel(TIM_INSTANCE_MCU_API, RF_API_TIMER_CHANNEL);
    _RF_API_disable_lr11xx_dio_irq();
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t* rx_data) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    RFE_status_t rfe_status = RFE_SUCCESS;
    sfx_bool dl_timeout = SIGFOX_FALSE;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (rx_data == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Reset flags.
    rf_api_ctx.flags.all = 0;
    (rx_data->data_received) = SIGFOX_FALSE;
    // Enable GPIO interrupt.
    status = _RF_API_enable_lr11xx_dio_irq(0b1 << LR11XX_IRQ_INDEX_RX_DONE);
    if (status != RF_API_SUCCESS) goto errors;
    // Start reception.
    lr11xx_status = LR11XX_set_mode(LR11XX_MODE_RX);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Wait for GPIO interrupt.
    while (rf_api_ctx.flags.dio_irq_flag == 0) {
        // Enter sleep mode.
        PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        IWDG_reload();
        // Check timeout.
        mcu_api_status = MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
        MCU_API_check_status((RF_API_status_t) RF_API_ERROR_DRIVER_MCU_API);
        // Exit if timeout.
        if (dl_timeout == SIGFOX_TRUE) {
            // Stop radio.
            lr11xx_status = LR11XX_set_mode(LR11XX_MODE_STANDBY_XOSC);
            LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
            // Exit loop.
            goto errors;
        }
    }
    // Read data.
    lr11xx_status = LR11XX_read_fifo(rf_api_ctx.dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
    // Read RSSI.
    rfe_status = RFE_get_rssi(LR11XX_RSSI_TYPE_AVERAGED, &(rf_api_ctx.dl_rssi_dbm));
    RFE_stack_exit_error(ERROR_BASE_RFE, (RF_API_status_t) RF_API_ERROR_DRIVER_RFE);
    // Update status flag.
    (rx_data->data_received) = SIGFOX_TRUE;
errors:
    // Disable GPIO interrupt.
    _RF_API_disable_lr11xx_dio_irq();
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8* dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16* dl_rssi_dbm) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if ((dl_phy_content == SIGFOX_NULL) || (dl_rssi_dbm == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_NULL_PARAMETER);
    }
    if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_BUFFER_SIZE);
    }
#endif
    // Fill data.
    for (idx = 0; idx < dl_phy_content_size; idx++) {
        dl_phy_content[idx] = rf_api_ctx.dl_phy_content[idx];
    }
    (*dl_rssi_dbm) = (sfx_s16) rf_api_ctx.dl_rssi_dbm;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_UNUSED(carrier_sense_params);
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32* latency_ms) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (latency_type >= RF_API_LATENCY_LAST) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) RF_API_ERROR_LATENCY_TYPE);
    }
#endif
    // Set latency.
    (*latency_ms) = RF_API_LATENCY_MS[latency_type];
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*******************************************************************/
RF_API_status_t RF_API_start_continuous_wave(void) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    LR11XX_status_t lr11xx_status = LR11XX_SUCCESS;
    // Start continuous wave.
    lr11xx_status = LR11XX_set_mode(LR11XX_MODE_TX_CW);
    LR11XX_stack_exit_error(ERROR_BASE_LR1110, (RF_API_status_t) RF_API_ERROR_DRIVER_LR11XX);
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    SIGFOX_UNUSED(version);
    SIGFOX_UNUSED(version_size_char);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
void RF_API_error(void) {
    // Force all front-end off.
    RF_API_de_init();
    RF_API_sleep();

}
#endif

#endif /* HW2_0 */
