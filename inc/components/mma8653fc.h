/*
 * mma8653fc.h
 *
 *  Created on: 1st may 2020
 *      Author: Ludo
 */

#ifndef __MMA8653FC_H__
#define __MMA8653FC_H__

#include "mma8653fc_reg.h"
#include "mode.h"

#if (defined SSM) || (defined ATM)

/*** MMA8653FC macros ***/

#define MMA8653FC_ACTIVE_CONFIG_LENGTH	10
#define MMA8653FC_SLEEP_CONFIG_LENGTH	3

/*** MMA8653FC structures ***/

typedef struct {
	unsigned char addr;
	unsigned char value;
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
unsigned char MMA8653FC_get_id(void);
void MMA8653FC_write_config(const MMA8653FC_register_setting_t* mma8653fc_config, unsigned char mma8653fc_config_size);
void MMA8653FC_get_data(signed int* x, signed int* y, signed int* z);
void MMA8653FC_set_motion_interrupt_flag(void);
void MMA8653FC_clear_motion_interrupt_flag(void);
unsigned char MMA8653FC_get_motion_interrupt_flag(void);

#endif

#endif /* __MMA8653FC_H__ */
