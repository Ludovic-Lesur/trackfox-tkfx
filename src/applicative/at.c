/*
 * at.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "at.h"

#include "adc.h"
#include "addon_sigfox_rf_protocol_api.h"
#include "aes.h"
#include "error.h"
#include "i2c.h"
#include "lpuart.h"
#include "lptim.h"
#include "math.h"
#include "mma8653fc.h"
#include "mode.h"
#include "neom8n.h"
#include "nvic.h"
#include "nvm.h"
#include "parser.h"
#include "pwr.h"
#include "rf_api.h"
#include "s2lp.h"
#include "sht3x.h"
#include "sigfox_api.h"
#include "sigfox_types.h"
#include "spi.h"
#include "string.h"
#include "types.h"
#include "usart.h"
#include "version.h"

#ifdef ATM

/*** AT local macros ***/

// Enabled commands.
#define AT_COMMANDS_SENSORS
#define AT_COMMANDS_GPS
#define AT_COMMANDS_NVM
#define AT_COMMANDS_SIGFOX
//#define AT_COMMANDS_TEST_MODES
// Commands.
#define AT_COMMAND_BUFFER_SIZE			128
// Parameters separator.
#define AT_CHAR_SEPARATOR				','
// Replies.
#define AT_REPLY_BUFFER_SIZE			128
#define AT_REPLY_END					"\r\n"
#define AT_REPLY_TAB					"     "
#define AT_STRING_VALUE_BUFFER_SIZE		16
// Duration of RSSI command.
#define AT_RSSI_REPORT_PERIOD_MS		500

/*** AT callbacks declaration ***/

static void _AT_print_ok(void);
static void _AT_print_command_list(void);
static void _AT_print_sw_version(void);
static void _AT_print_error_stack(void);
#ifdef AT_COMMANDS_SENSORS
static void _AT_adc_callback(void);
static void _AT_ths_callback(void);
static void _AT_acc_callback(void);
#endif
#ifdef AT_COMMANDS_GPS
static void _AT_gps_callback(void);
#endif
#ifdef AT_COMMANDS_NVM
static void _AT_nvmr_callback(void);
static void _AT_nvm_callback(void);
static void _AT_get_id_callback(void);
static void _AT_set_id_callback(void);
static void _AT_get_key_callback(void);
static void _AT_set_key_callback(void);
#endif
#ifdef AT_COMMANDS_SIGFOX
static void _AT_so_callback(void);
static void _AT_sb_callback(void);
static void _AT_sf_callback(void);
#endif
#ifdef AT_COMMANDS_TEST_MODES
static void _AT_tm_callback(void);
static void _AT_cw_callback(void);
static void _AT_dl_callback(void);
static void _AT_rssi_callback(void);
#endif

/*** AT local structures ***/

typedef struct {
	PARSER_mode_t mode;
	char_t* syntax;
	char_t* parameters;
	char_t* description;
	void (*callback)(void);
} AT_command_t;

typedef struct {
	// Command.
	volatile char_t command[AT_COMMAND_BUFFER_SIZE];
	volatile uint32_t command_size;
	volatile uint8_t line_end_flag;
	PARSER_context_t parser;
	// Reply.
	char_t reply[AT_REPLY_BUFFER_SIZE];
	uint32_t reply_size;
	// Sigfox RC.
	sfx_rc_t sigfox_rc;
} AT_context_t;

/*** AT local global variables ***/

static const AT_command_t AT_COMMAND_LIST[] = {
	{PARSER_MODE_COMMAND, "AT", STRING_NULL, "Ping command", _AT_print_ok},
	{PARSER_MODE_COMMAND, "AT?", STRING_NULL, "List all available AT commands", _AT_print_command_list},
	{PARSER_MODE_COMMAND, "AT$V?", STRING_NULL, "Get SW version", _AT_print_sw_version},
	{PARSER_MODE_COMMAND, "AT$ERROR?", STRING_NULL, "Read error stack", _AT_print_error_stack},
	{PARSER_MODE_COMMAND, "AT$RST", STRING_NULL, "Reset MCU", PWR_software_reset},
#ifdef AT_COMMANDS_SENSORS
	{PARSER_MODE_COMMAND, "AT$ADC?", STRING_NULL, "Get ADC measurements", _AT_adc_callback},
	{PARSER_MODE_COMMAND, "AT$THS?", STRING_NULL, "Get temperature and humidity (SHT30)", _AT_ths_callback},
	{PARSER_MODE_COMMAND, "AT$ACC?", STRING_NULL, "Read accelerometer chip ID (MMA8653FC)", _AT_acc_callback},
#endif
#ifdef AT_COMMANDS_GPS
	{PARSER_MODE_HEADER,  "AT$GPS=", "timeout[s]", "Get GPS position (NEOM8N)", _AT_gps_callback},
#endif
#ifdef AT_COMMANDS_NVM
	{PARSER_MODE_COMMAND, "AT$NVMR", STRING_NULL, "Reset NVM data", _AT_nvmr_callback},
	{PARSER_MODE_HEADER,  "AT$NVM=", "address[dec]", "Get NVM data", _AT_nvm_callback},
	{PARSER_MODE_COMMAND, "AT$ID?", STRING_NULL, "Get Sigfox device ID", _AT_get_id_callback},
	{PARSER_MODE_HEADER,  "AT$ID=", "id[hex]", "Set Sigfox device ID", _AT_set_id_callback},
	{PARSER_MODE_COMMAND, "AT$KEY?", STRING_NULL, "Get Sigfox device key", _AT_get_key_callback},
	{PARSER_MODE_HEADER,  "AT$KEY=", "key[hex]", "Set Sigfox device key", _AT_set_key_callback},
#endif
#ifdef AT_COMMANDS_SIGFOX
	{PARSER_MODE_COMMAND, "AT$SO", STRING_NULL, "Sigfox send control message", _AT_so_callback},
	{PARSER_MODE_HEADER,  "AT$SB=", "data[bit],(bidir_flag[bit])", "Sigfox send bit", _AT_sb_callback},
	{PARSER_MODE_HEADER,  "AT$SF=", "data[hex],(bidir_flag[bit])", "Sigfox send frame", _AT_sf_callback},
#endif
#ifdef AT_COMMANDS_TEST_MODES
	{PARSER_MODE_HEADER,  "AT$TM=", "rc_index[dec],test_mode[dec]", "Execute Sigfox test mode", _AT_tm_callback},
	{PARSER_MODE_HEADER,  "AT$CW=", "frequency[hz],enable[bit],(output_power[dbm])", "Start or stop continuous radio transmission", _AT_cw_callback},
	{PARSER_MODE_HEADER,  "AT$DL=", "frequency[hz]", "Continuous downlink frames decoding", _AT_dl_callback},
	{PARSER_MODE_HEADER,  "AT$RSSI=", "frequency[hz],duration[s]", "Start or stop continuous RSSI measurement", _AT_rssi_callback},
#endif
};

static AT_context_t at_ctx;

/*** AT local functions ***/

/* GENERIC MACRO TO ADD A CHARACTER TO THE REPLY BUFFER.
 * @param character:	Character to add.
 * @return:				None.
 */
#define _AT_reply_add_char(character) { \
	at_ctx.reply[at_ctx.reply_size] = character; \
	at_ctx.reply_size = (at_ctx.reply_size + 1) % AT_REPLY_BUFFER_SIZE; \
}

/* APPEND A STRING TO THE REPONSE BUFFER.
 * @param tx_string:	String to add.
 * @return:				None.
 */
static void _AT_reply_add_string(char_t* tx_string) {
	// Fill TX buffer with new bytes.
	while (*tx_string) {
		_AT_reply_add_char(*(tx_string++));
	}
}

/* APPEND A VALUE TO THE REPONSE BUFFER.
 * @param tx_value:		Value to add.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @return:				None.
 */
static void _AT_reply_add_value(int32_t tx_value, STRING_format_t format, uint8_t print_prefix) {
	// Local variables.
	STRING_status_t string_status = STRING_SUCCESS;
	char_t str_value[AT_STRING_VALUE_BUFFER_SIZE];
	uint8_t idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_STRING_VALUE_BUFFER_SIZE ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	string_status = STRING_value_to_string(tx_value, format, print_prefix, str_value);
	STRING_error_check();
	// Add string.
	_AT_reply_add_string(str_value);
}

/* SEND AT REPONSE OVER AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_reply_send(void) {
	// Local variables.
	USART_status_t usart_status = USART_SUCCESS;
	// Add ending string.
	_AT_reply_add_string(AT_REPLY_END);
	_AT_reply_add_char(STRING_CHAR_NULL);
	// Send response over UART.
	usart_status = USART2_send_string(at_ctx.reply);
	USART_error_check();
	// Flush reply buffer.
	at_ctx.reply_size = 0;
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_ok(void) {
	_AT_reply_add_string("OK");
	_AT_reply_send();
}

/* PRINT AN ERROR THROUGH AT INTERFACE.
 * @param error:	Error code to print.
 * @return:			None.
 */
static void _AT_print_error(ERROR_t error) {
	// Add error to stack.
	ERROR_stack_add(error);
	// Print error.
	_AT_reply_add_string("ERROR_");
	if (error < 0x0100) {
		_AT_reply_add_value(0, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 0);
	}
	else {
		_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
	}
	_AT_reply_send();
}

/* PRINT ALL SUPPORTED AT COMMANDS.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_command_list(void) {
	// Local variables.
	uint32_t idx = 0;
	// Commands loop.
	for (idx=0 ; idx<(sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)) ; idx++) {
		// Print syntax.
		_AT_reply_add_string(AT_COMMAND_LIST[idx].syntax);
		// Print parameters.
		_AT_reply_add_string(AT_COMMAND_LIST[idx].parameters);
		_AT_reply_send();
		// Print description.
		_AT_reply_add_string(AT_REPLY_TAB);
		_AT_reply_add_string(AT_COMMAND_LIST[idx].description);
		_AT_reply_send();
	}
	_AT_print_ok();
}

/* PRINT SW VERSION.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_sw_version(void) {
	_AT_reply_add_string("SW");
	_AT_reply_add_value((int32_t) GIT_MAJOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(".");
	_AT_reply_add_value((int32_t) GIT_MINOR_VERSION, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string(".");
	_AT_reply_add_value((int32_t) GIT_COMMIT_INDEX, STRING_FORMAT_DECIMAL, 0);
	if (GIT_DIRTY_FLAG != 0) {
		_AT_reply_add_string(".d");
	}
	_AT_reply_add_string(" (");
	_AT_reply_add_value((int32_t) GIT_COMMIT_ID, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_reply_add_string(")");
	_AT_reply_send();
	_AT_print_ok();
}

/* PRINT ERROR STACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_print_error_stack(void) {
	// Local variables.
	ERROR_t error = SUCCESS;
	// Read stack.
	if (ERROR_stack_is_empty() != 0) {
		_AT_reply_add_string("Error stack empty");
	}
	else {
		// Unstack all errors.
		_AT_reply_add_string("[ ");
		do {
			error = ERROR_stack_read();
			if (error != SUCCESS) {
				_AT_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
				_AT_reply_add_string(" ");
			}
		}
		while (error != SUCCESS);
		_AT_reply_add_string("]");
	}
	_AT_reply_send();
	_AT_print_ok();
}

#ifdef AT_COMMANDS_SENSORS
/* AT$ADC? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_adc_callback(void) {
	// Local variables.
	ADC_status_t adc1_status = ADC_SUCCESS;
	uint32_t voltage_mv = 0;
	int8_t tmcu_degrees = 0;
	// Trigger internal ADC conversions.
	_AT_reply_add_string("ADC running...");
	_AT_reply_send();
	adc1_status = ADC1_power_on();
	ADC1_error_check_print();
	adc1_status = ADC1_perform_measurements();
	ADC1_error_check_print();
	// Read and print data.
	// Source voltage.
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VSRC_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_reply_add_string("Vsrc=");
	_AT_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	// Supercap voltage.
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VCAP_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_reply_add_string("mV Vcap=");
	_AT_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	// MCU voltage.
	adc1_status = ADC1_get_data(ADC_DATA_INDEX_VMCU_MV, &voltage_mv);
	ADC1_error_check_print();
	_AT_reply_add_string("mV Vmcu=");
	_AT_reply_add_value((int32_t) voltage_mv, STRING_FORMAT_DECIMAL, 0);
	// MCU temperature.
	adc1_status = ADC1_get_tmcu(&tmcu_degrees);
	ADC1_error_check_print();
	_AT_reply_add_string("mV Tmcu=");
	_AT_reply_add_value((int32_t) tmcu_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dC");
	_AT_reply_send();
	_AT_print_ok();
errors:
	ADC1_power_off();
	return;
}

/* AT$THS? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_ths_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	SHT3X_status_t sht3x_status = SHT3X_SUCCESS;
	int8_t tamb_degrees = 0;
	uint8_t hamb_percent = 0;
	// Perform measurements.
	i2c1_status = I2C1_power_on();
	I2C1_error_check_print();
	_AT_reply_add_string("SHT3X running...");
	_AT_reply_send();
	sht3x_status = SHT3X_perform_measurements(SHT3X_I2C_ADDRESS);
	SHT3X_error_check_print();
	// Read data.
	sht3x_status = SHT3X_get_temperature(&tamb_degrees);
	SHT3X_error_check_print();
	sht3x_status = SHT3X_get_humidity(&hamb_percent);
	SHT3X_error_check_print();
	// Print results.
	_AT_reply_add_string("T=");
	_AT_reply_add_value((int32_t) tamb_degrees, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dC H=");
	_AT_reply_add_value((int32_t) hamb_percent, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("%");
	_AT_reply_send();
	_AT_print_ok();
errors:
	I2C1_power_off();
	return;
}

/* PRINT ACCELEROMETER DATA ON USART.
 * @param:	None.
 * @return:	None.
 */
static void _AT_acc_callback(void) {
	// Local variables.
	I2C_status_t i2c1_status = I2C_SUCCESS;
	MMA8653FC_status_t mma8653fc_status = MMA8653FC_SUCCESS;
	uint8_t chip_id = 0;
	// Get ID.
	i2c1_status = I2C1_power_on();
	I2C1_error_check_print();
	mma8653fc_status = MMA8653FC_get_id(&chip_id);
	MMA8653FC_error_check_print();
	// Print data.
	_AT_reply_add_string("MMA8653FC chip ID: ");
	_AT_reply_add_value(chip_id, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_reply_send();
	_AT_print_ok();
errors:
	I2C1_power_off();
	return;
}
#endif

#ifdef AT_COMMANDS_GPS
/* AT$GPS EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_gps_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NEOM8N_status_t neom8n_status = NEOM8N_SUCCESS;
	LPUART_status_t lpuart1_status = LPUART_SUCCESS;
	int32_t timeout_seconds = 0;
	uint32_t fix_duration_seconds = 0;
	NEOM8N_position_t gps_position;
	// Read timeout parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &timeout_seconds);
	PARSER_error_check_print();
	// Power on GPS.
	lpuart1_status = LPUART1_power_on();
	LPUART1_error_check_print();
	// Start GPS fix.
	_AT_reply_add_string("GPS running...");
	_AT_reply_send();
	neom8n_status = NEOM8N_get_position(&gps_position, (uint32_t) timeout_seconds, 0, &fix_duration_seconds);
	NEOM8N_error_check_print();
	// Latitude.
	_AT_reply_add_string("Lat=");
	_AT_reply_add_value((gps_position.lat_degrees), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("d");
	_AT_reply_add_value((gps_position.lat_minutes), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("'");
	_AT_reply_add_value((gps_position.lat_seconds), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("''");
	_AT_reply_add_string(((gps_position.lat_north_flag) == 0) ? "S" : "N");
	// Longitude.
	_AT_reply_add_string(" Long=");
	_AT_reply_add_value((gps_position.long_degrees), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("d");
	_AT_reply_add_value((gps_position.long_minutes), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("'");
	_AT_reply_add_value((gps_position.long_seconds), STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("''");
	_AT_reply_add_string(((gps_position.long_east_flag) == 0) ? "W" : "E");
	// Altitude.
	_AT_reply_add_string(" Alt=");
	_AT_reply_add_value((gps_position.altitude), STRING_FORMAT_DECIMAL, 0);
	// Fix duration.
	_AT_reply_add_string("m Fix=");
	_AT_reply_add_value(fix_duration_seconds, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("s");
	_AT_reply_send();
	_AT_print_ok();
errors:
	LPUART1_power_off();
	return;
}
#endif

#ifdef AT_COMMANDS_NVM
/* AT$NVMR EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_nvmr_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	// Reset all NVM field to default value.
	nvm_status = NVM_reset_default();
	NVM_error_check_print();
	_AT_print_ok();
errors:
	return;
}

/* AT$NVM EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_nvm_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	int32_t address = 0;
	uint8_t nvm_data = 0;
	// Read address parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &address);
	PARSER_error_check_print();
	// Read byte at requested address.
	nvm_status = NVM_read_byte((uint16_t) address, &nvm_data);
	NVM_error_check_print();
	// Print data.
	_AT_reply_add_value(nvm_data, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_reply_send();
	_AT_print_ok();
errors:
	return;
}

/* AT$ID? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_get_id_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t idx = 0;
	uint8_t id_byte = 0;
	// Retrieve device ID in NVM.
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), &id_byte);
		NVM_error_check_print();
		_AT_reply_add_value(id_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	_AT_reply_send();
	_AT_print_ok();
errors:
	return;
}

/* AT$ID EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_set_id_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t device_id[ID_LENGTH];
	uint8_t extracted_length = 0;
	uint8_t idx = 0;
	// Read ID parameter.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, ID_LENGTH, 1, device_id, &extracted_length);
	PARSER_error_check_print();
	// Write device ID in NVM.
	for (idx=0 ; idx<ID_LENGTH ; idx++) {
		nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_ID + ID_LENGTH - idx - 1), device_id[idx]);
		NVM_error_check_print();
	}
	_AT_print_ok();
errors:
	return;
}

/* AT$KEY? EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_get_key_callback(void) {
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t idx = 0;
	uint8_t key_byte = 0;
	// Retrieve device key in NVM.
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		nvm_status = NVM_read_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), &key_byte);
		NVM_error_check_print();
		_AT_reply_add_value(key_byte, STRING_FORMAT_HEXADECIMAL, (idx==0 ? 1 : 0));
	}
	_AT_reply_send();
	_AT_print_ok();
errors:
	return;
}

/* AT$KEY EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_set_key_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t device_key[AES_BLOCK_SIZE];
	uint8_t extracted_length = 0;
	uint8_t idx = 0;
	// Read key parameter.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, AES_BLOCK_SIZE, 1, device_key, &extracted_length);
	PARSER_error_check_print();
	// Write device ID in NVM.
	for (idx=0 ; idx<AES_BLOCK_SIZE ; idx++) {
		nvm_status = NVM_write_byte((NVM_ADDRESS_SIGFOX_DEVICE_KEY + idx), device_key[idx]);
		NVM_error_check_print();
	}
	_AT_print_ok();
errors:
	return;
}
#endif

#ifdef AT_COMMANDS_SIGFOX
/* PRINT SIGFOX DOWNLINK DATA ON AT INTERFACE.
 * @param dl_payload:	Downlink data to print.
 * @return:				None.
 */
static void _AT_print_dl_payload(sfx_u8* dl_payload) {
	_AT_reply_add_string("+RX=");
	uint8_t idx = 0;
	for (idx=0 ; idx<SIGFOX_DOWNLINK_DATA_SIZE_BYTES ; idx++) {
		_AT_reply_add_value(dl_payload[idx], STRING_FORMAT_HEXADECIMAL, 0);
	}
	_AT_reply_send();
}

/* AT$SO EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_so_callback(void) {
	// Local variables.
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	// Send Sigfox OOB frame.
	sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
	SIGFOX_API_error_check_print();
	_AT_reply_add_string("Sigfox library running...");
	_AT_reply_send();
	sigfox_api_status = SIGFOX_API_send_outofband(SFX_OOB_SERVICE);
	SIGFOX_API_error_check_print();
	_AT_print_ok();
errors:
	sigfox_api_status = SIGFOX_API_close();
	SIGFOX_API_error_check();
	return;
}

/* AT$SB EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_sb_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int32_t data = 0;
	int32_t bidir_flag = 0;
	sfx_u8 dl_payload[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
	// First try with 2 parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, &data);
	if (parser_status == PARSER_SUCCESS) {
		// Try parsing downlink request parameter.
		parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &bidir_flag);
		PARSER_error_check_print();
		// Send Sigfox bit with specified downlink request.
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		_AT_reply_add_string("Sigfox library running...");
		_AT_reply_send();
		sigfox_api_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, (sfx_bool) bidir_flag);
		SIGFOX_API_error_check_print();
		if (bidir_flag != SFX_FALSE) {
			_AT_print_dl_payload(dl_payload);
		}
	}
	else {
		// Try with 1 parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &data);
		PARSER_error_check_print();
		// Send Sigfox bit with no downlink request (by default).
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		_AT_reply_add_string("Sigfox library running...");
		_AT_reply_send();
		sigfox_api_status = SIGFOX_API_send_bit((sfx_bool) data, dl_payload, 2, 0);
		SIGFOX_API_error_check_print();
	}
	_AT_print_ok();
errors:
	sigfox_api_status = SIGFOX_API_close();
	SIGFOX_API_error_check();
	return;
}

/* AT$SF EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_sf_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	sfx_u8 data[SIGFOX_UPLINK_DATA_MAX_SIZE_BYTES];
	uint8_t extracted_length = 0;
	int32_t bidir_flag = 0;
	sfx_u8 dl_payload[SIGFOX_DOWNLINK_DATA_SIZE_BYTES];
	// First try with 2 parameters.
	parser_status = PARSER_get_byte_array(&at_ctx.parser, AT_CHAR_SEPARATOR, 12, 0, data, &extracted_length);
	if (parser_status == PARSER_SUCCESS) {
		// Try parsing downlink request parameter.
		parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &bidir_flag);
		PARSER_error_check_print();
		// Send Sigfox frame with specified downlink request.
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		_AT_reply_add_string("Sigfox library running...");
		_AT_reply_send();
		sigfox_api_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, bidir_flag);
		SIGFOX_API_error_check_print();
		if (bidir_flag != 0) {
			_AT_print_dl_payload(dl_payload);
		}
	}
	else {
		// Try with 1 parameter.
		parser_status = PARSER_get_byte_array(&at_ctx.parser, STRING_CHAR_NULL, 12, 0, data, &extracted_length);
		PARSER_error_check_print();
		// Send Sigfox frame with no downlink request (by default).
		sigfox_api_status = SIGFOX_API_open(&at_ctx.sigfox_rc);
		SIGFOX_API_error_check_print();
		_AT_reply_add_string("Sigfox library running...");
		_AT_reply_send();
		sigfox_api_status = SIGFOX_API_send_frame(data, extracted_length, dl_payload, 2, 0);
		SIGFOX_API_error_check_print();
	}
	_AT_print_ok();
errors:
	sigfox_api_status = SIGFOX_API_close();
	SIGFOX_API_error_check();
	return;
}
#endif

#ifdef AT_COMMANDS_TEST_MODES
/* PRINT SIGFOX DOWNLINK FRAME ON AT INTERFACE.
 * @param dl_payload:	Downlink data to print.
 * @return:				None.
 */
static void _AT_print_dl_phy_content(sfx_u8* dl_phy_content, int32_t rssi_dbm) {
	_AT_reply_add_string("+DL_PHY=");
	uint8_t idx = 0;
	for (idx=0 ; idx<SIGFOX_DOWNLINK_PHY_SIZE_BYTES ; idx++) {
		_AT_reply_add_value(dl_phy_content[idx], STRING_FORMAT_HEXADECIMAL, 0);
	}
	_AT_reply_add_string(" RSSI=");
	_AT_reply_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
	_AT_reply_add_string("dBm");
	_AT_reply_send();
}

/* AT$TM EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_tm_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int32_t rc_index = 0;
	int32_t test_mode = 0;
	// Read RC parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &rc_index);
	PARSER_error_check_print();
	// Read test mode parameter.
	parser_status =  PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &test_mode);
	PARSER_error_check_print();
	// Call test mode function wth public key.
	_AT_reply_add_string("Sigfox addon running...");
	_AT_reply_send();
	sigfox_api_status = ADDON_SIGFOX_RF_PROTOCOL_API_test_mode((sfx_rc_enum_t) rc_index, (sfx_test_mode_t) test_mode);
	SIGFOX_API_error_check_print();
	_AT_print_ok();
errors:
	return;
}

/* AT$CW EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_cw_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int32_t enable = 0;
	int32_t frequency_hz = 0;
	int32_t power_dbm = 0;
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &frequency_hz);
	PARSER_error_check_print();
	// First try with 3 parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, AT_CHAR_SEPARATOR, &enable);
	if (parser_status == PARSER_SUCCESS) {
		// There is a third parameter, try to parse power.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &power_dbm);
		PARSER_error_check_print();
		// CW with given output power.
		SIGFOX_API_stop_continuous_transmission();
		if (enable != 0) {
			sigfox_api_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
			SIGFOX_API_error_check_print();
			s2lp_status = S2LP_set_rf_output_power((int8_t) power_dbm);
			S2LP_error_check_print();
			_AT_reply_add_string("S2LP running...");
			_AT_reply_send();
		}
	}
	else {
		// Power is not given, try to parse enable as last parameter.
		parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_BOOLEAN, STRING_CHAR_NULL, &enable);
		PARSER_error_check_print();
		// CW with last output power.
		SIGFOX_API_stop_continuous_transmission();
		if (enable != 0) {
			sigfox_api_status = SIGFOX_API_start_continuous_transmission((sfx_u32) frequency_hz, SFX_NO_MODULATION);
			SIGFOX_API_error_check_print();
			_AT_reply_add_string("S2LP running...");
			_AT_reply_send();
		}
	}
	_AT_print_ok();
	return;
errors:
	sigfox_api_status = SIGFOX_API_stop_continuous_transmission();
	SIGFOX_API_error_check();
	return;
}

/* AT$DL EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_dl_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	sfx_u8 dl_phy_content[SIGFOX_DOWNLINK_PHY_SIZE_BYTES];
	sfx_s16 rssi_dbm = 0;
	sfx_rx_state_enum_t dl_status = DL_PASSED;
	int32_t frequency_hz = 0;
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &frequency_hz);
	PARSER_error_check_print();
	// Start radio.
	sigfox_api_status = RF_API_init(SFX_RF_MODE_RX);
	SIGFOX_API_error_check_print();
	sigfox_api_status = RF_API_change_frequency(frequency_hz);
	SIGFOX_API_error_check_print();
	_AT_reply_add_string("RX GFSK running...");
	_AT_reply_send();
	while (dl_status == DL_PASSED) {
		sigfox_api_status = RF_API_wait_frame(dl_phy_content, &rssi_dbm, &dl_status);
		SIGFOX_API_error_check_print();
		// Check result.
		if (dl_status == DL_PASSED) {
			_AT_print_dl_phy_content(dl_phy_content, rssi_dbm);
		}
		else {
			_AT_reply_add_string("RX timeout");
			_AT_reply_send();
		}
	}
	_AT_print_ok();
errors:
	sigfox_api_status = RF_API_stop();
	SIGFOX_API_error_check();
	return;
}

/* AT$RSSI EXECUTION CALLBACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_rssi_callback(void) {
	// Local variables.
	PARSER_status_t parser_status = PARSER_ERROR_UNKNOWN_COMMAND;
	S2LP_status_t s2lp_status = S2LP_SUCCESS;
	LPTIM_status_t lptim1_status = LPTIM_SUCCESS;
	sfx_error_t sigfox_api_status = SFX_ERR_NONE;
	int32_t frequency_hz = 0;
	int32_t duration_s = 0;
	int16_t rssi_dbm = 0;
	uint32_t report_loop = 0;
	// Read frequency parameter.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, AT_CHAR_SEPARATOR, &frequency_hz);
	PARSER_error_check_print();
	// Read duration parameters.
	parser_status = PARSER_get_parameter(&at_ctx.parser, STRING_FORMAT_DECIMAL, STRING_CHAR_NULL, &duration_s);
	PARSER_error_check_print();
	// Init radio.
	sigfox_api_status = RF_API_init(SFX_RF_MODE_RX);
	SIGFOX_API_error_check_print();
	sigfox_api_status = RF_API_change_frequency((sfx_u32) frequency_hz);
	SIGFOX_API_error_check_print();
	// Start continuous listening.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_READY);
	S2LP_error_check_print();
	s2lp_status = S2LP_wait_for_state(S2LP_STATE_READY);
	S2LP_error_check_print();
	// Start radio.
	s2lp_status = S2LP_send_command(S2LP_COMMAND_RX);
	S2LP_error_check_print();
	// Measurement loop.
	_AT_reply_add_string("S2LP running...");
	_AT_reply_send();
	while (report_loop < ((duration_s * 1000) / AT_RSSI_REPORT_PERIOD_MS)) {
		// Read RSSI.
		s2lp_status = S2LP_get_rssi(S2LP_RSSI_TYPE_RUN, &rssi_dbm);
		S2LP_error_check_print();
		// Print RSSI.
		_AT_reply_add_string("RSSI=");
		_AT_reply_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
		_AT_reply_add_string("dBm");
		_AT_reply_send();
		// Report delay.
		lptim1_status = LPTIM1_delay_milliseconds(AT_RSSI_REPORT_PERIOD_MS, 0);
		LPTIM1_error_check_print();
		report_loop++;
	}
	_AT_print_ok();
errors:
	sigfox_api_status = RF_API_stop();
	SIGFOX_API_error_check();
	return;
}
#endif

/* RESET AT PARSER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_reset_parser(void) {
	// Flush buffers.
	at_ctx.command_size = 0;
	at_ctx.reply_size = 0;
	// Reset flag.
	at_ctx.line_end_flag = 0;
	// Reset parser.
	at_ctx.parser.buffer = (char_t*) at_ctx.command;
	at_ctx.parser.buffer_size = 0;
	at_ctx.parser.separator_idx = 0;
	at_ctx.parser.start_idx = 0;
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_decode(void) {
	// Local variables.
	uint8_t idx = 0;
	uint8_t decode_success = 0;
	// Update parser length.
	at_ctx.parser.buffer_size = at_ctx.command_size;
	// Loop on available commands.
	for (idx=0 ; idx<(sizeof(AT_COMMAND_LIST) / sizeof(AT_command_t)) ; idx++) {
		// Check type.
		if (PARSER_compare(&at_ctx.parser, AT_COMMAND_LIST[idx].mode, AT_COMMAND_LIST[idx].syntax) == PARSER_SUCCESS) {
			// Execute callback and exit.
			AT_COMMAND_LIST[idx].callback();
			decode_success = 1;
			break;
		}
	}
	if (decode_success == 0) {
		_AT_print_error(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
		goto errors;
	}
errors:
	_AT_reset_parser();
	return;
}

/*** AT functions ***/

/* INIT AT MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_init(void) {
	// Init context.
	_AT_reset_parser();
	at_ctx.sigfox_rc = (sfx_rc_t) RC1;
	// Enable USART.
	USART2_enable_interrupt();
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_task(void) {
	// Trigger decoding function if line end found.
	if (at_ctx.line_end_flag != 0) {
		// Decode and execute command.
		USART2_disable_interrupt();
		_AT_decode();
		USART2_enable_interrupt();
	}
}

/* FILL AT COMMAND BUFFER WITH A NEW BYTE (CALLED BY USART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void AT_fill_rx_buffer(uint8_t rx_byte) {
	// Append byte if line end flag is not allready set.
	if (at_ctx.line_end_flag == 0) {
		// Check ending characters.
		if ((rx_byte == STRING_CHAR_CR) || (rx_byte == STRING_CHAR_LF)) {
			at_ctx.command[at_ctx.command_size] = STRING_CHAR_NULL;
			at_ctx.line_end_flag = 1;
		}
		else {
			// Store new byte.
			at_ctx.command[at_ctx.command_size] = rx_byte;
			// Manage index.
			at_ctx.command_size = (at_ctx.command_size + 1) % AT_COMMAND_BUFFER_SIZE;
		}
	}
}

/* PRINT SIGFOX LIBRARY RESULT.
 * @param test_result:	Test result.
 * @param rssi:			Downlink signal rssi in dBm.
 */
void AT_print_test_result(uint8_t test_result, int32_t rssi_dbm) {
	// Check result.
	if (test_result == 0) {
		_AT_reply_add_string("Test failed.");
	}
	else {
		_AT_reply_add_string("Test passed. RSSI=");
		_AT_reply_add_value(rssi_dbm, STRING_FORMAT_DECIMAL, 0);
		_AT_reply_add_string("dBm");
	}
	_AT_reply_send();
}

#endif
