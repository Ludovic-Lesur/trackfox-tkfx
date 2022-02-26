/*
 * spi.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef SPI_H
#define SPI_H

/*** SPI functions ***/

void SPI1_init(void);
void SPI1_enable(void);
void SPI1_disable(void);
void SPI1_power_on(void);
void SPI1_power_off(void);
unsigned char SPI1_write_byte(unsigned char tx_data);
unsigned char SPI1_read_byte(unsigned char tx_data, unsigned char* rx_data);

#endif /* SPI_H_ */
