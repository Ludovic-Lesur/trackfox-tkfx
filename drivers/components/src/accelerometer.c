/*
 * accelerometer.c
 *
 *  Created on: 16 nov. 2024
 *      Author: Ludo
 */

#include "accelerometer.h"

#include "fxls89xxxx.h"
#include "mma865xfc.h"

/*** ACCELEROMETER global variables ***/

const ACCELEROMETER_register_setting_t ACCELEROMETER_CONFIGURATION_ACTIVE[ACCELEROMETER_CONFIGURATION_SIZE_ACTIVE] = {
#ifdef HW2_0

#else
    { MMA865XFC_REGISTER_CTRL_REG1, 0x00 }, // ACTIVE='0' (standby mode required to program registers).
    { MMA865XFC_REGISTER_XYZ_DATA_CFG, 0x00 }, // Full scale = +/-2g.
    { MMA865XFC_REGISTER_CTRL_REG2, 0x1B }, // (S)MODS='11' (low power operation) and SLPE='0' (Auto sleep disabled).
    { MMA865XFC_REGISTER_CTRL_REG3, 0x0A }, // WAKE_FF_MT='1' (motion interrupt wakes the sensor) and IPOL='1' (interrupt pin active high).
    { MMA865XFC_REGISTER_FF_MT_CFG, 0x78 }, // OAE='1' (motion detection). ELE='0' (latch disabled, bit automatically cleared). XEFE=YEFE=ZEFE='1' (any direction enabled).
    { MMA865XFC_REGISTER_FF_MT_THS, 0x91 }, // DBCNTM='1' and threshold value (1.071g).
    { MMA865XFC_REGISTER_FF_MT_COUNT, 0x00 }, // Debouncing counter not used.
    { MMA865XFC_REGISTER_CTRL_REG5, 0x04 }, // INT_CFG_FF_MT='1' (motion interrupt on INT1 pin).
    { MMA865XFC_REGISTER_CTRL_REG4, 0x04 }, // INT_EN_FF_MT='1' (motion interrupt enabled).
    { MMA865XFC_REGISTER_CTRL_REG1, 0x39 } // DR='111' (1Hz) and ACTIVE='1'.
#endif
};

const ACCELEROMETER_register_setting_t ACCELEROMETER_CONFIGURATION_SLEEP[ACCELEROMETER_CONFIGURATION_SIZE_SLEEP] = {
#ifdef HW2_0

#else
    { MMA865XFC_REGISTER_CTRL_REG2, 0x5B }, // RESET='1'.
    { MMA865XFC_REGISTER_CTRL_REG2, 0x1B }, // RESET='0', (S)MODS='11' (low power operation) and SLPE='0' (auto sleep disabled).
    { MMA865XFC_REGISTER_CTRL_REG3, 0x02 }, // IPOL='1' (interrupt pin active high).
#endif
};
