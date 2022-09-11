/*
 * spi.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __SPI_H__
#define __SPI_H__

/*** SPI functions ***/

void SPI1_init(void);
void SPI1_enable(void);
void SPI1_disable(void);
void SPI1_power_on(void);
void SPI1_power_off(void);
unsigned char SPI1_write_byte(unsigned char tx_data);
unsigned char SPI1_read_byte(unsigned char tx_data, unsigned char* rx_data);

#endif /* __SPI_H__ */
