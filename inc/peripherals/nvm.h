/*
 * nvm.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __NVM_H__
#define __NVM_H__

/*** NVM macros ***/

// Sigfox device parameters.
#define NVM_ADDRESS_SIGFOX_DEVICE_ID				0
#define NVM_ADDRESS_SIGFOX_DEVICE_KEY				4
#define NVM_ADDRESS_SIGFOX_PN				20
#define NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER				22
#define NVM_ADDRESS_FH				24
#define NVM_ADDRESS_SIGFOX_FH				26
// Device configuration (mapped on downlink frame).
#define NVM_ADDRESS_DEVICE_CONFIGURATION				27

/*** NVM functions ***/

void NVM_enable(void);
void NVM_disable(void);
void NVM_read_byte(unsigned short address_offset, unsigned char* byte_to_read);
void NVM_write_byte(unsigned short address_offset, unsigned char byte_to_store);
void NVM_reset_default(void);

#endif /* __NVM_H__ */
