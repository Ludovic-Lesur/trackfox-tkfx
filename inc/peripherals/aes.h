/*
 * aes.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __AES_H__
#define __AES_H__

/*** AES macros ***/

#define AES_BLOCK_SIZE 	16 // 128-bits is 16 bytes.

/*** AES functions ***/

void AES_init(void);
void AES_disable(void);
void AES_encrypt(unsigned char data_in[AES_BLOCK_SIZE], unsigned char data_out[AES_BLOCK_SIZE], unsigned char init_vector[AES_BLOCK_SIZE], unsigned char key[AES_BLOCK_SIZE]);

#endif /* __AES_H___ */
