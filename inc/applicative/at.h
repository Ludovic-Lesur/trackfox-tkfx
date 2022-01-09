/*
 * at.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef AT_H
#define AT_H

#include "mode.h"

#ifdef ATM

/*** AT macros ***/

// Enabled commands.
#define AT_COMMANDS_GPS
#define AT_COMMANDS_SENSORS
#define AT_COMMANDS_NVM
#define AT_COMMANDS_SIGFOX
#define AT_COMMANDS_CW
#define AT_COMMANDS_TEST_MODES

/*** AT functions ***/

void AT_Init(void);
void AT_Task(void);
void AT_FillRxBuffer(unsigned char rx_byte);
void AT_PrintTestResult(unsigned char status, int rssi);

#endif

#endif /* AT_H */
