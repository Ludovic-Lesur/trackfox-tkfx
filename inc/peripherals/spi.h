/*
 * spi.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef SPI_H
#define SPI_H

/*** SPI functions ***/

void SPI1_Init(void);
void SPI1_Enable(void);
void SPI1_Disable(void);
void SPI1_PowerOn(void);
void SPI1_PowerOff(void);
unsigned char SPI1_WriteByte(unsigned char tx_data);
unsigned char SPI1_ReadByte(unsigned char tx_data, unsigned char* rx_data);

#endif /* SPI_H_ */
