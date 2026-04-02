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
    { FXLS89XXXX_REGISTER_SENS_CONFIG1, 0x00 }, // // Full scale = +/-2g.
    { FXLS89XXXX_REGISTER_SENS_CONFIG3, 0x99 }, // ODR = 6.25Hz.
    { FXLS89XXXX_REGISTER_SDCD_UTHS_MSB, 0x00 }, // High threshold delta = +0.1g.
    { FXLS89XXXX_REGISTER_SDCD_UTHS_LSB, 0x66 },
    { FXLS89XXXX_REGISTER_SDCD_LTHS_MSB, 0x0F }, // Low threshold delta = -0.1g.
    { FXLS89XXXX_REGISTER_SDCD_LTHS_LSB, 0x9A },
    { FXLS89XXXX_REGISTER_INT_EN, 0x20 }, // Enable outside threshold interrupt.
    { FXLS89XXXX_REGISTER_SENS_CONFIG4, 0x01 }, // INT polarity is active high.
    { FXLS89XXXX_REGISTER_INT_PIN_SEL, 0x00 }, // Use INT1 pin.
    { FXLS89XXXX_REGISTER_SDCD_CONFIG1, 0xB8 }, // Enable event latch and interrupts on all axis.
    { FXLS89XXXX_REGISTER_SDCD_CONFIG2, 0xD8 }, // Enable SDCD, enable relative mode and disable debouncing.
    { FXLS89XXXX_REGISTER_SENS_CONFIG1, 0x01 } // ACTIVE='1'.
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
    { FXLS89XXXX_REGISTER_SENS_CONFIG1, 0x00 }, // ACTIVE='0'.
#else
    { MMA865XFC_REGISTER_CTRL_REG2, 0x5B }, // RESET='1'.
    { MMA865XFC_REGISTER_CTRL_REG2, 0x1B }, // RESET='0', (S)MODS='11' (low power operation) and SLPE='0' (auto sleep disabled).
    { MMA865XFC_REGISTER_CTRL_REG3, 0x02 }, // IPOL='1' (interrupt pin active high).
#endif
};
