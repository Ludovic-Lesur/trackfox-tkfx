/*
 * math.h
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#ifndef __MATH_H__
#define __MATH_H__

/*** MATH functions ***/

unsigned int MATH_pow_10(unsigned char power);
unsigned int MATH_average(unsigned int* data, unsigned char data_length);
unsigned int MATH_median_filter(unsigned int* data, unsigned char median_length, unsigned char average_length);
signed int MATH_two_complement(unsigned int value, unsigned char sign_bit_position);

#endif /* __MATH_H__ */
