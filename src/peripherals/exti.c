/*
 * exti.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "exti.h"

#include "exti_reg.h"
#include "mapping.h"
#include "mma8653fc.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "syscfg_reg.h"

/*** EXTI local functions ***/

/* EXTI LINES 0-1 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI0_1_IRQHandler(void) {
	// Accelero IRQ (PA1).
	if (((EXTI -> PR) & (0b1 << (GPIO_ACCELERO_IRQ.gpio_num))) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_ACCELERO_IRQ.gpio_num)); // PIFx='1' (writing '1' clears the bit).
		// Set motion interrupt flag.
		MMA8653FC_SetMotionInterruptFlag();
	}
}

/* EXTI LINES 2-3 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI2_3_IRQHandler(void) {
	// Can be used for TP1 (PA4).
}

/* EXTI LINES 4-15 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void EXTI4_15_IRQHandler(void) {
	// S2LP GPIOs (PA10/PA11).
	if (((EXTI -> PR) & (0b1 << (GPIO_S2LP_GPIO0.gpio_num))) != 0) {
		// Clear flag.
		EXTI -> PR |= (0b1 << (GPIO_S2LP_GPIO0.gpio_num)); // PIFx='1' (writing '1' clears the bit).
	}
}

/*** EXTI functions ***/

/* INIT EXTI PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void EXTI_Init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	// Disable interrupts by default.
	NVIC_DisableInterrupt(IT_EXTI_0_1);
	NVIC_DisableInterrupt(IT_EXTI_2_3);
	NVIC_DisableInterrupt(IT_EXTI_4_15);
	// Clear all flags.
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
}

/* CONFIGURE A GPIO AS EXTERNAL INTERRUPT SOURCE.
 * @param gpio:		GPIO to be attached to EXTI peripheral.
 * @edge_trigger:	Interrupt edge trigger (see EXTI_Trigger egpio_numeration in exti.h).
 * @return:			None.
 */
void EXTI_ConfigureInterrupt(const GPIO* gpio, EXTI_Trigger edge_trigger) {
	// Select GPIO port.
	SYSCFG -> EXTICR[((gpio -> gpio_num) / 4)] &= ~(0b1111 << (4 * ((gpio -> gpio_num) % 4)));
	SYSCFG -> EXTICR[((gpio -> gpio_num) / 4)] |= ((gpio -> gpio_port_index) << (4 * ((gpio -> gpio_num) % 4)));
	// Select triggers.
	switch (edge_trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> gpio_num))); // IMx='1'.
		EXTI -> RTSR |= (0b1 << ((gpio -> gpio_num))); // Rising edge enabled.
		EXTI -> FTSR &= ~(0b1 << ((gpio -> gpio_num))); // Falling edge disabled.
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> gpio_num))); // IMx='1'.
		EXTI -> RTSR &= ~(0b1 << ((gpio -> gpio_num))); // Rising edge disabled.
		EXTI -> FTSR |= (0b1 << ((gpio -> gpio_num))); // Falling edge enabled.
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		EXTI -> IMR |= (0b1 << ((gpio -> gpio_num))); // IMx='1'.
		EXTI -> RTSR |= (0b1 << ((gpio -> gpio_num))); // Rising edge enabled.
		EXTI -> FTSR |= (0b1 << ((gpio -> gpio_num))); // Falling edge enabled.
		break;
	// Unknown configuration.
	default:
		EXTI -> IMR &= ~(0b1 << ((gpio -> gpio_num))); // IMx='0'.
		EXTI -> RTSR &= ~(0b1 << ((gpio -> gpio_num))); // Rising edge disabled.
		EXTI -> FTSR &= ~(0b1 << ((gpio -> gpio_num))); // Falling edge disabled.
		break;
	}
}

/* CLEAR ALL EXTI FLAGS.
 * @param:	None.
 * @return:	None.
 */
void EXTI_ClearAllFlags(void) {
	// Clear all flags.
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
}
