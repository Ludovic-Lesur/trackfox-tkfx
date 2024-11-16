/*
 * i2c_address.h
 *
 *  Created on: 12 nov. 2024
 *      Author: Ludo
 */

#ifndef __I2C_ADDRESS_H__
#define __I2C_ADDRESS_H__

/*!******************************************************************
 * \enum I2C_address_mapping_t
 * \brief I2C slaves address mapping.
 *******************************************************************/
typedef enum {
    I2C_ADDRESS_SHT30 = 0x44,
    I2C_ADDRESS_MMA8653FC = 0x1D
} I2C_address_mapping_t;

#endif /* __I2C_ADDRESS_H__ */
