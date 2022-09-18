/*
 * math.h
 *
 *  Created on: 28 aug. 2020
 *      Author: Ludo
 */

#ifndef __MATH_H__
#define __MATH_H__

#include "mode.h"

/*** MATH macros ***/

#define MATH_BINARY_MAX_LENGTH			32
#define MATH_DECIMAL_MAX_LENGTH			10
#define MATH_HEXADECIMAL_MAX_LENGTH		4
#define MATH_BYTE_MAX					0xFF

/*** MATH structures ***/

typedef enum {
	MATH_SUCCESS = 0,
	MATH_ERROR_OVERFLOW,
	MATH_ERROR_UNDEFINED,
	MATH_ERROR_SIGN_BIT,
	MATH_ERROR_BASE_LAST = 0x0100
} MATH_status_t;

/*** MATH functions ***/

unsigned char MATH_min_u8(unsigned char* data, unsigned char data_length);
unsigned short MATH_min_u16(unsigned short* data, unsigned char data_length);
unsigned int MATH_min_u32(unsigned int* data, unsigned char data_length);

unsigned char MATH_max_u8(unsigned char* data, unsigned char data_length);
unsigned short MATH_max_u16(unsigned short* data, unsigned char data_length);
unsigned int MATH_max_u32(unsigned int* data, unsigned char data_length);

unsigned char MATH_average_u8(unsigned char* data, unsigned char data_length);
unsigned short MATH_average_u16(unsigned short* data, unsigned char data_length);
unsigned int MATH_average_u32(unsigned int* data, unsigned char data_length);

unsigned char MATH_median_filter_u8(unsigned char* data, unsigned char median_length, unsigned char average_length);
unsigned short MATH_median_filter_u16(unsigned short* data, unsigned char median_length, unsigned char average_length);
unsigned int MATH_median_filter_u32(unsigned int* data, unsigned char median_length, unsigned char average_length);

MATH_status_t MATH_pow_10(unsigned char power, unsigned int* result);
unsigned int MATH_abs(signed int x);
MATH_status_t MATH_atan2(signed int x, signed int y, unsigned int* alpha);

MATH_status_t MATH_two_complement(unsigned int value, unsigned char sign_bit_position, signed int* result);
MATH_status_t MATH_one_complement(signed int value, unsigned char sign_bit_position, unsigned int* result);

#define MATH_status_check(error_base) { if (math_status != MATH_SUCCESS) { status = error_base + math_status; goto errors; }}
#define MATH_error_check() { ERROR_status_check(math_status, MATH_SUCCESS, ERROR_BASE_MATH); }
#define MATH_error_check_print() { ERROR_status_check_print(math_status, MATH_SUCCESS, ERROR_BASE_MATH); }

#endif /* __MATH_H__ */
