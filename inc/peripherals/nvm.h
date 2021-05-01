/*
 * nvm.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef NVM_H
#define NVM_H

/*** NVM macros ***/

// Sigfox device parameters.
#define NVM_SIGFOX_ID_ADDRESS_OFFSET				0
#define NVM_SIGFOX_KEY_ADDRESS_OFFSET				4
#define NVM_SIGFOX_PN_ADDRESS_OFFSET				20
#define NVM_SIGFOX_SEQ_ADDRESS_OFFSET				22
#define NVM_SIGFOX_FH_ADDRESS_OFFSET				24
#define NVM_SIGFOX_RL_ADDRESS_OFFSET				26
// Device configuration (mapped on downlink frame).
#define NVM_CONFIG_START_ADDRESS_OFFSET				27

/*** NVM functions ***/

void NVM_Enable(void);
void NVM_Disable(void);
void NVM_ReadByte(unsigned short address_offset, unsigned char* byte_to_read);
void NVM_WriteByte(unsigned short address_offset, unsigned char byte_to_store);
void NVM_ResetDefault(void);

#endif /* NVM_H */
