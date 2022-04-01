/*
 * nvm.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "nvm.h"

#include "flash_reg.h"
#include "rcc_reg.h"

/*** NVM local functions ***/

/* UNLOCK NVM.
 * @param:	None.
 * @return:	None.
 */
static void NVM_unlock(void) {
	// Check no write/erase operation is running.
	while (((FLASH -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Check the NVM is not allready unlocked.
	if (((FLASH -> PECR) & (0b1 << 0)) != 0) {
		// Perform unlock sequence.
		FLASH -> PEKEYR = 0x89ABCDEF;
		FLASH -> PEKEYR = 0x02030405;
	}
}

/* LOCK NVM.
 * @param:	None.
 * @return:	None.
 */
static void NVM_lock(void) {
	// Check no write/erase operation is running.
	while (((FLASH -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock PECR register.
	FLASH -> PECR |= (0b1 << 0); // PELOCK='1'.
}

/*** NVM functions ***/

/* ENABLE NVM INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void NVM_enable(void) {
	// Enable NVM peripheral.
	RCC -> AHBENR |= (0b1 << 8); // MIFEN='1'.
}

/* DISABLE NVM INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void NVM_disable(void) {
	// Disable NVM peripheral.
	RCC -> AHBENR &= ~(0b1 << 8); // MIFEN='1'.
}

/* READ A BYTE STORED IN NVM.
 * @param address_offset:	Address offset starting from NVM start address (expressed in bytes).
 * @param byte_to_read:		Pointer to byte that will contain the value to read.
 * @return:					None.
 */
void NVM_read_byte(unsigned short address_offset, unsigned char* byte_to_read) {
	// Unlock NVM.
	NVM_unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*byte_to_read) = *((unsigned char*) (EEPROM_START_ADDRESS+address_offset)); // Read byte at requested address.
	}
	// Lock NVM.
	NVM_lock();
}

/* WRITE A BYTE TO NVM.
 * @param address_offset:	Address offset starting from NVM start address (expressed in bytes).
 * @param byte_to_store:	Byte to store in NVM.
 * @return:					None.
 */
void NVM_write_byte(unsigned short address_offset, unsigned char byte_to_store) {
	// Unlock NVM.
	NVM_unlock();
	// Check if address is in EEPROM range.
	if (address_offset < EEPROM_SIZE) {
		(*((unsigned char*) (EEPROM_START_ADDRESS+address_offset))) = byte_to_store; // Write byte to requested address.
	}
	// Wait end of operation.
	while (((FLASH -> SR) & (0b1 << 0)) != 0); // Wait till BSY='1'.
	// Lock NVM.
	NVM_lock();
}

/* RESET ALL NVM FIELDS TO DEFAULT VALUE.
 * @param:	None.
 * @return:	None.
 */
void NVM_reset_default(void) {
	// Sigfox parameters.
	NVM_write_byte((NVM_ADDRESS_SIGFOX_PN + 0), 0x00);
	NVM_write_byte((NVM_ADDRESS_SIGFOX_PN + 1), 0x00);
	NVM_write_byte((NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER + 0), 0x00);
	NVM_write_byte((NVM_ADDRESS_SIGFOX_MESSAGE_COUNTER + 1), 0x00);
	NVM_write_byte((NVM_ADDRESS_FH + 0), 0x00);
	NVM_write_byte((NVM_ADDRESS_FH + 1), 0x00);
	NVM_write_byte(NVM_ADDRESS_SIGFOX_FH, 0x00);
	// Device configuration (mapped on downlink frame).
	// TBD.
}
