/*
 * string.h
 *
 *  Created on: 05 dec. 2021
 *      Author: Ludo
 */

#ifndef __STRING_H__
#define __STRING_H__

/*** STRING macros ***/

#define STRING_CHAR_NULL	'\0'
#define STRING_CHAR_CR		'\r'
#define STRING_CHAR_LF		'\n'
#define STRING_CHAR_MINUS	'-'
#define STRING_CHAR_DOT		'.'

/*** STRING structures ***/

typedef enum {
	STRING_FORMAT_BOOLEAN,
	STRING_FORMAT_HEXADECIMAL,
	STRING_FORMAT_DECIMAL,
	STRING_FORMAT_ASCII,
	STRING_FORMAT_LAST
} STRING_format_t;

/*** STRING functions ***/

unsigned char STRING_char_to_value(char ascii_code);
char STRING_decimal_to_ascii(unsigned char decimal_digit);
char STRING_hexa_to_ascii(unsigned char hexa_digit);
unsigned char STRING_is_hexadecimal_char(char ascii_code);
unsigned char STRING_is_decimal_char(char ascii_code);
void STRING_value_to_string(int value, STRING_format_t format, unsigned char print_prefix, char* string);

#endif /* __STRING_H__ */
