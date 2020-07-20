/*
 * mma8653fc.h
 *
 *  Created on: 1st may 2020
 *      Author: Ludo
 */

#ifndef MMA8653FC_H
#define MMA8653FC_H

#include "mma8653fc_reg.h"

/*** MMA8653FC structures ***/

typedef struct {
	unsigned char mma8653fc_reg_addr;
	unsigned char mma8653fc_reg_value;
} MMA8653FC_RegisterSetting;

static const MMA8653FC_RegisterSetting mma8653_tkfx_config[] = {
	{MMA8653FC_REG_CTRL_REG1, 0x00}, // ACTIVE='0' (standby mode required to program registers).
	{MMA8653FC_REG_CTRL_REG2, 0x1B}, // (S)MODS='11' (low power operation) and SLPE='0' (Auto sleep disabled).
	{MMA8653FC_REG_CTRL_REG3, 0x0A}, // WAKE_FF_MT='1' (motion interrupt wakes the sensor) and IPOL='1' (interrupt pin active high).
	{MMA8653FC_REG_FF_MT_CFG, 0x78}, // OAE='1' (motion detection). ELE='0' (latch disabled, bit automatically cleared). XEFE=YEFE=ZEFE='1' (any direction enabled).
	{MMA8653FC_REG_FF_MT_THS, 0x10}, // DBCNTM='0' and threshold value (to be tuned).
	{MMA8653FC_REG_FF_MT_COUNT, 0x0A}, // Debouncing counter (to be tuned).
	{MMA8653FC_REG_CTRL_REG5, 0x04}, // INT_CFG_FF_MT='1' (motion interrupt on INT1 pin).
	{MMA8653FC_REG_CTRL_REG4, 0x04}, // INT_EN_FF_MT='1' (motion interrupt enabled).
	{MMA8653FC_REG_CTRL_REG1, 0x39} // DR='111' (1Hz) and ACTIVE='1'.
};

/*** MMA8653 macros ***/

#define MMA8653FC_TKFX_CONFIG_SIZE	(sizeof(mma8653_tkfx_config) / sizeof(MMA8653FC_RegisterSetting))

/*** MMA8653FC functions ***/

void MMA8653FC_Init(void);
void MMA8653FC_WriteConfig(const MMA8653FC_RegisterSetting* mma8653fc_config, unsigned char mma8653fc_config_size);
void MMA8653FC_SetMotionInterruptFlag(void);
void MMA8653FC_ClearMotionInterruptFlag(void);
unsigned char MMA8653FC_GetMotionInterruptFlag(void);

#endif /* MMA8653FC_H */
