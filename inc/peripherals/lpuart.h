/*
 * lpuart.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef LPUART_H
#define LPUART_H

/*** LPUART functions ***/

void LPUART1_Init(unsigned char lpuart_use_lse);
void LPUART1_UpdateBrr(void);
void LPUART1_EnableTx(void);
void LPUART1_EnableRx(void);
void LPUART1_Disable(void);
void LPUART1_PowerOn(void);
void LPUART1_PowerOff(void);
void LPUART1_SendByte(unsigned char byte_to_send);

#endif /* LPUART_H */
