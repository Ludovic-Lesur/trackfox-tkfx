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
#include "mode.h"

#if (defined SSM) || (defined ATM)

/*** MMA8653 local macros ***/

#define MMA8653FC_I2C_ADDRESS	0x1D

/*** MMA8653FC local global variables ***/

volatile unsigned char mma8653fc_motion_interrupt_flag = 0;

/*** MMA8653 local functions ***/

/* COMPUTE THE TWO'S COMPLEMENT OF A GIVEN VALUE.
 * @param value:				Value on which the two's complement has to be computed.
 * @param sign_bit_position:	Position of the sign bit.
 * @return result:				Result of computation.
 */
static signed int MMA8653FC_ComputeTwoComplement(unsigned int value, unsigned char sign_bit_position) {
	signed int result = 0;
	// Check sign bit.
	if ((value & (0b1 << sign_bit_position)) == 0) {
		// Value is positive: nothing to do.
		result = value;
	}
	else {
		// Value is negative.
		unsigned char bit_idx = 0;
		unsigned int not_value = 0;
		for (bit_idx=0 ; bit_idx<=sign_bit_position ; bit_idx++) {
			if ((value & (0b1 << bit_idx)) == 0) {
				not_value |= (0b1 << bit_idx);
			}
		}
		unsigned int absolute_value = not_value + 1;
		result = (-1) * absolute_value;
	}
	return result;
}

/*** MMA8653FC functions ***/

/* INIT MMA8653FC INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void MMA8653FC_Init(void) {
	// Init flag.
	mma8653fc_motion_interrupt_flag = 0;
	// Configure interrupt pin.
	GPIO_Configure(&GPIO_ACCELERO_IRQ, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	EXTI_ConfigureGpio(&GPIO_ACCELERO_IRQ, EXTI_TRIGGER_RISING_EDGE);
}

/* READ SENSOR ID.
 * @param:	None.
 * @return:	None.
 */
unsigned char MMA8653FC_GetId(void) {
	unsigned char who_am_i = 0;
	unsigned char local_addr = MMA8653FC_REG_WHO_AM_I;
	unsigned char i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	i2c_access = I2C1_Read(MMA8653FC_I2C_ADDRESS, &who_am_i, 1);
	return who_am_i;
}

/* CONFIGURE THE ACCELEROMETER.
 * @param mma8653fc_config:			Registers setting array.
 * @param mma8653fc_config_size:	Length of the config array.
 * @return:							None.
 */
void MMA8653FC_WriteConfig(const MMA8653FC_RegisterSetting* mma8653fc_config, unsigned char mma8653fc_config_size) {
	unsigned char i2c_access = 0;
	unsigned char i2c_tx_data[2];
	unsigned char reg_idx = 0;
	for (reg_idx=0 ; reg_idx<mma8653fc_config_size ; reg_idx++) {
		i2c_tx_data[0] = (mma8653fc_config[reg_idx].mma8653fc_reg_addr);
		i2c_tx_data[1] = (mma8653fc_config[reg_idx].mma8653fc_reg_value);
		i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, i2c_tx_data, 2, 1);
		if (i2c_access == 0) break;
	}
}

/* READ ACCELERATION DATA.
 * @param x:	Pointer to signed integer that will contain X-axis acceleration.
 * @param y:	Pointer to signed integer that will contain Y-axis acceleration.
 * @param z:	Pointer to signed integer that will contain Z-axis acceleration.
 * @return:		None.
 */
void MMA8653FC_GetData(signed int* x, signed int* y, signed int* z) {
	unsigned int data = 0;
	unsigned char reg_data = 0;
	unsigned char local_addr = 0;
	unsigned char i2c_access = 0;
	// X-axis.
	local_addr = MMA8653FC_REG_OUT_X_MSB;
	i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_Read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_X_LSB;
	i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_Read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data |= (reg_data >> 6);
	(*x) = MMA8653FC_ComputeTwoComplement(data, 9);
	// Y-axis.
	local_addr = MMA8653FC_REG_OUT_Y_MSB;
	i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_Read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Y_LSB;
	i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_Read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data |= (reg_data >> 6);
	(*y) = MMA8653FC_ComputeTwoComplement(data, 9);
	// Z-axis.
	local_addr = MMA8653FC_REG_OUT_Z_MSB;
	i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_Read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data = 0;
	data |= reg_data << 2;
	local_addr = MMA8653FC_REG_OUT_Z_LSB;
	i2c_access = I2C1_Write(MMA8653FC_I2C_ADDRESS, &local_addr, 1, 0);
	if (i2c_access == 0) return;
	i2c_access = I2C1_Read(MMA8653FC_I2C_ADDRESS, &reg_data, 1);
	if (i2c_access == 0) return;
	data |= (reg_data >> 6);
	(*z) = MMA8653FC_ComputeTwoComplement(data, 9);
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
 * @return mma8653fc_motion_interrupt_flag:	'1' if the motion interrupt occurred (EXTI line), '0' otherwise.
 */
unsigned char MMA8653FC_GetMotionInterruptFlag(void) {
	return mma8653fc_motion_interrupt_flag;
}

#endif
