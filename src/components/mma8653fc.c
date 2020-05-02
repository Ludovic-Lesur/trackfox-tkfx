/*
 * mma8653fc.c
 *
 *  Created on: 1st may 2020
 *      Author: Ludo
 */

#include "mma8653fc.h"

#include "exti.h"
#include "gpio.h"
#include "i2c.h"
#include "mapping.h"

/*** MMA8653 local macros ***/

#define MMA8653FC_I2C_ADDRESS	0x1D

/*** MMA8653FC local global variables ***/

volatile unsigned char mma8653fc_motion_interrupt_flag = 0;

/*** MMA8653FC functions ***/

/* INIT MMA8653FC INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void MMA8653FC_Init(void) {
	// Init flag.
	mma8653fc_motion_interrupt_flag = 0;
	// Configure interrupt pin.
	GPIO_Configure(&GPIO_ACCELERO_IRQ, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	EXTI_ConfigureInterrupt(&GPIO_ACCELERO_IRQ, EXTI_TRIGGER_RISING_EDGE);
}

/* CONFIGURE THE ACCELEROMETER.
 * @param mma8653fc_config:			Registers setting array.
 * @param mma8653fc_config_size:	Length of the config array.
 * @return:							None.
 */
void MMA8653FC_WriteConfig(const MMA8653FC_RegisterSetting* mma8653fc_config, unsigned char mma8653fc_config_size) {
	// Registers write loop.
	unsigned char reg_idx = 0;
	unsigned char i2c_tx_data[2];
	unsigned char i2c_access = 0;
	for (reg_idx=0 ; reg_idx<mma8653fc_config_size ; reg_idx++) {
		i2c_tx_data[0] = (mma8653fc_config -> mma8653fc_reg_addr);
		i2c_tx_data[1] = (mma8653fc_config -> mma8653fc_reg_value);
		i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, i2c_tx_data, 2);
		if (i2c_access == 0) break;
	}
}

/* SET MOTION INTERRUPT STATUS.
 * @param:	None.
 * @return:	None.
 */
void MMA8653FC_SetMotionInterruptFlag(void) {
	mma8653fc_motion_interrupt_flag = 1;
}

/* CLEAR MOTION INTERRUPT STATUS.
 * @param:	None.
 * @return:	None.
 */
void MMA8653FC_ClearMotionInterruptFlag(void) {
	mma8653fc_motion_interrupt_flag = 0;
}

/* GET MOTION INTERRUPT STATUS.
 * @param:									None.
 * @return mma8653fc_motion_interrupt_flag:	'1' if the motion interrupt occured (EXTI line), '0' otherwise.
 */
unsigned char MMA8653FC_GetMotionInterruptFlag(void) {
	return mma8653fc_motion_interrupt_flag;
}
