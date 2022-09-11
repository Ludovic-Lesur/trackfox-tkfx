/*
 * i2c.h
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#ifndef __I2C_H__
#define __I2C_H__

/*** I2C functions ***/

void I2C1_init(void);
void I2C1_disable(void);
void I2C1_power_on(void);
void I2C1_power_off(void);
unsigned char I2C1_write(unsigned char slave_address, unsigned char* tx_buf, unsigned char tx_buf_length, unsigned char stop_flag);
unsigned char I2C1_read(unsigned char slave_address, unsigned char* rx_buf, unsigned char rx_buf_length);

#endif /* __I2C_H__ */
