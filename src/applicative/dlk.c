/*
 * dlk.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludovic
 */

#include "dlk.h"

#include "nvm.h"
#include "sigfox_api.h"

/*** DLK local macros ***/

/*** DLK local structures ***/

/* READ THE CURRENT CONFIGURATION FROM NVM.
 * @param config:	Pointer to DLK_Parameters strcutres that will contain current configuration.
 * @return:			None.
 */
void DLK_Read(DLK_Parameters* config) {
	// TBD.
}

/* WRITE A NEW CONFIGURATION FROM NVM.
 * @param downlink_data:	Raw downlink received from Sigfox network.
 * @return:					None.
 */
void DLK_Write(unsigned char* downlink_data) {
	// Fill NVM with new data.
	unsigned char byte_idx = 0;
	for (byte_idx=0 ; byte_idx<SFX_DOWNLINK_DATA_SIZE_BYTES ; byte_idx++) {
		NVM_WriteByte((NVM_CONFIG_START_ADDRESS_OFFSET + byte_idx), downlink_data[byte_idx]);
	}
}
