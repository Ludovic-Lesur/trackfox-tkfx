/*
 * math.h
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#ifndef MATH_H
#define MATH_H

/*** MATH functions ***/

unsigned int MATH_pow_10(unsigned char power);
unsigned int MATH_average(unsigned int* data, unsigned char data_length);
unsigned int MATH_median_filter(unsigned int* data, unsigned char median_length, unsigned char average_length);
signed int MATH_two_complement(unsigned int value, unsigned char sign_bit_position);

#endif /* MATH_H */
