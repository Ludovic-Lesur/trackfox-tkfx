/*
 * mma8653fc.h
 *
 *  Created on: 1st may 2020
 *      Author: Ludo
 */

#ifndef __MMA8653FC_H__
#define __MMA8653FC_H__

#include "i2c.h"
#include "math.h"
#include "mma8653fc_reg.h"
#include "types.h"

/*** MMA8653FC macros ***/

#define MMA8653FC_I2C_ADDRESS			0x1D
#define MMA8653FC_ACTIVE_CONFIG_LENGTH	10
#define MMA8653FC_SLEEP_CONFIG_LENGTH	3

/*** MMA8653FC structures ***/

typedef enum {
	MMA8653FC_SUCCESS = 0,
	MMA8653FC_ERROR_NULL_PARAMETER,
	MMA8653FC_ERROR_CONFIG_SIZE,
	MMA8653FC_ERROR_BASE_I2C = 0x0100,
	MMA8653FC_ERROR_BASE_MATH = (MMA8653FC_ERROR_BASE_I2C + I2C_ERROR_BASE_LAST),
	MMA8653FC_ERROR_BASE_LAST = (MMA8653FC_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} MMA8653FC_status_t;

typedef struct {
	uint8_t addr;
	uint8_t value;
} MMA8653FC_register_setting_t;

static const MMA8653FC_register_setting_t mma8653_active_config[MMA8653FC_ACTIVE_CONFIG_LENGTH] = {
	{MMA8653FC_REG_CTRL_REG1, 0x00}, // ACTIVE='0' (standby mode required to program registers).
	{MMA8653FC_REG_XYZ_DATA_CFG, 0x00}, // Full scale = +/-2g.
	{MMA8653FC_REG_CTRL_REG2, 0x1B}, // (S)MODS='11' (low power operation) and SLPE='0' (Auto sleep disabled).
	{MMA8653FC_REG_CTRL_REG3, 0x0A}, // WAKE_FF_MT='1' (motion interrupt wakes the sensor) and IPOL='1' (interrupt pin active high).
	{MMA8653FC_REG_FF_MT_CFG, 0x78}, // OAE='1' (motion detection). ELE='0' (latch disabled, bit automatically cleared). XEFE=YEFE=ZEFE='1' (any direction enabled).
	{MMA8653FC_REG_FF_MT_THS, 0x91}, // DBCNTM='1' and threshold value (1.071g).
	{MMA8653FC_REG_FF_MT_COUNT, 0x00}, // Debouncing counter not used.
	{MMA8653FC_REG_CTRL_REG5, 0x04}, // INT_CFG_FF_MT='1' (motion interrupt on INT1 pin).
	{MMA8653FC_REG_CTRL_REG4, 0x04}, // INT_EN_FF_MT='1' (motion interrupt enabled).
	{MMA8653FC_REG_CTRL_REG1, 0x39} // DR='111' (1Hz) and ACTIVE='1'.
};

static const MMA8653FC_register_setting_t mma8653_sleep_config[MMA8653FC_SLEEP_CONFIG_LENGTH] = {
	{MMA8653FC_REG_CTRL_REG2, 0x5B}, // RESET='1'.
	{MMA8653FC_REG_CTRL_REG2, 0x1B}, // RESET='0', (S)MODS='11' (low power operation) and SLPE='0' (auto sleep disabled).
	{MMA8653FC_REG_CTRL_REG3, 0x02}, // IPOL='1' (interrupt pin active high).
};

/*** MMA8653FC functions ***/

void MMA8653FC_init(void);
MMA8653FC_status_t MMA8653FC_get_id(uint8_t* chip_id);
MMA8653FC_status_t MMA8653FC_write_config(const MMA8653FC_register_setting_t* mma8653fc_config, uint8_t mma8653fc_config_size);
MMA8653FC_status_t MMA8653FC_get_data(int32_t* x, int32_t* y, int32_t* z);
void MMA8653FC_set_motion_interrupt_flag(void);
void MMA8653FC_clear_motion_interrupt_flag(void);
uint8_t MMA8653FC_get_motion_interrupt_flag(void);

#define MMA8653FC_status_check(error_base) { if (mma8653fc_status != MMA8653FC_SUCCESS) { status = error_base + mma8653fc_status; goto errors; }}
#define MMA8653FC_error_check() { ERROR_status_check(mma8653fc_status, MMA8653FC_SUCCESS, ERROR_BASE_MMA8653FC); }
#define MMA8653FC_error_check_print() { ERROR_status_check_print(mma8653fc_status, MMA8653FC_SUCCESS, ERROR_BASE_MMA8653FC); }

#endif /* __MMA8653FC_H__ */
