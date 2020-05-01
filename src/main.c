/*
 * main.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

// Registers
#include "rcc_reg.h"
// Peripherals.
#include "adc.h"
#include "aes.h"
#include "dma.h"
#include "exti.h"
#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "lptim.h"
#include "lpuart.h"
#include "mapping.h"
#include "nvic.h"
#include "nvm.h"
#include "pwr.h"
#include "rcc.h"
#include "spi.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
// Components.
#include "neom8n.h"
#include "s2lp.h"
#include "sht3x.h"
#include "sigfox_types.h"
// Applicative.
#include "at.h"
#include "dlk.h"
#include "geoloc.h"
#include "mode.h"
#include "monitoring.h"
#include "sigfox_api.h"

#ifdef SSM
/* MAIN FUNCTION FOR START/STOP MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Init memory */
	NVIC_Init();
	FLASH_Init();
	//NVM_Enable();

	/* Init GPIO (required for clock tree configuration) */
	GPIO_Init();

	/* Init clocks */
	RCC_Init();
	RCC_SwitchToHsi();
	//RTC_Reset();
	//RCC_EnableLsi();
	// High speed oscillator.
	//if (RCC_SwitchToHse() == 0) {
		//RCC_SwitchToHsi();
	//}

	/* Init peripherals */
	//unsigned int lsi_frequency_hz = RCC_GetLsiFrequency();
	// Timers.
	TIM21_Init();
	TIM22_Init();
	TIM21_Start();
	TIM22_Start();
	LPTIM1_Init(0);
	// RTC.
	//RTC_Init(0, lsi_frequency_hz);
	// DMA.
	DMA1_Init();
	// Analog.
	ADC1_Init();
	// External interrupts.
	EXTI_Init();
	// Communication interfaces.
	LPUART1_Init();
	USART2_Init();
	I2C1_Init();
	SPI1_Init();
	// Hardware AES.
	AES_Init();

	/* Init components */
	NEOM8N_Init();
	SHT3X_Init();
	S2LP_Init();

	/* CW test */
	SPI1_Enable();
	SPI1_PowerOn();
	S2LP_ConfigureChargePump();
	S2LP_SetModulation(S2LP_MODULATION_NONE);
	S2LP_SetRfFrequency(867000000);
	S2LP_SendCommand(S2LP_CMD_LOCKTX);
	LPTIM1_DelayMilliseconds(1);
	S2LP_SendCommand(S2LP_CMD_TX);

	/* Main loop */
	while (1) {

	}

	return 0;
}
#endif

#ifdef ATM
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {

	/* Init memory */
	NVIC_Init();
	FLASH_Init();
	NVM_Enable();

	/* Init GPIO (required for clock tree configuration) */
	GPIO_Init();

	/* Init clocks */
	RCC_Init();
	RTC_Reset();
	RCC_EnableLsi();
	// High speed oscillator.
	if (RCC_SwitchToHse() == 0) {
		RCC_SwitchToHsi();
	}

	/* Init peripherals */
	unsigned int lsi_frequency_hz = RCC_GetLsiFrequency();
	// Timers.
	TIM21_Init();
	TIM22_Init();
	TIM21_Start();
	TIM22_Start();
	LPTIM1_Init(0);
	// RTC.
	RTC_Init(0, lsi_frequency_hz);
	// DMA.
	DMA1_Init();
	// Analog.
	ADC1_Init();
	// External interrupts.
	EXTI_Init();
	// Communication interfaces.
	LPUART1_Init();
	USART2_Init();
	I2C1_Init();
	SPI1_Init();
	// Hardware AES.
	AES_Init();

	/* Init components */
	NEOM8N_Init();
	SHT3X_Init();
	S2LP_Init();

	/* Init applicative layers */
	AT_Init();

	/* Main loop */
	while (1) {
		// Perform AT commands parsing.
		AT_Task();
	}

	return 0;
}
#endif
