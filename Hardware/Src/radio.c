/*
 * radio.c
 */

#include "radio.h"
#include "string.h"

#define RADIO_UART_TIMEOUT_MS 100

#define RADIO_HEADER 0xBEEF

/**
 * @brief Sends command to radio to adjust a setting
 * @param[in] dev: Radio device
 * @param[in] cmd: 2-letter command to send
 * @param[in] param: Parameter following command
 */
static void radio_send_cmd(radio_t* dev, const char* cmd, const char* param) {
	int cmd_len = strlen(cmd);
	int param_len = strlen(param);

	// Set command
	char tx_data[2 + cmd_len + 1 + param_len + 1];
	memcpy(&tx_data, "AT", 2);
	memcpy(&tx_data[2], cmd, cmd_len);

	// Add space
	memcpy(&tx_data[2 + cmd_len], " ", 1);

	// Add parameter
	memcpy(&tx_data[3 + cmd_len], param, param_len);

	// Add carriage return
	memcpy(&tx_data[3 + cmd_len + param_len], "\r", 1);

	// Transmit
	HAL_UART_Transmit(dev->huart, (uint8_t*) tx_data, 4 + cmd_len + param_len, RADIO_UART_TIMEOUT_MS);
	uint8_t rx_data;
	HAL_UART_Receive(dev->huart, &rx_data, 1, 1000);
}

void radio_init(radio_t* dev, UART_HandleTypeDef* huart) {
	// Check user inputs
	if (!dev || !huart) {
		return;
	}

	// Set initial dev properties
	dev->huart = huart;

	// Assume most configurations are set up beforehand via XCTU, except those explicitly set below

	// Set device to transparent operating mode (AP = 0) to define our own protocol
	radio_send_cmd(dev, "AP", "0");
	// Set packetization timeout (number of characters of silence to wait before sending packet) to 3
	radio_send_cmd(dev, "RO", "3");
	// Exit command mode
	radio_send_cmd(dev, "CN\r", "");
}

void radio_transmit(radio_t* dev, uint8_t* data, uint16_t len) {
	// Check user input
	if (!dev || !data) {
		return;
	}

	// Add custom protocol of packet (2-byte header ID, 2-byte length, & 1-byte checksum)
	uint8_t tx_data[len + 5];
	// Header
	tx_data[0] = RADIO_HEADER >> 8;
	tx_data[1] = RADIO_HEADER & 0xFF;
	// Length
	tx_data[2] = len >> 8;
	tx_data[3] = len & 0xFF;
	// Data
	memcpy(&tx_data[4], data, len);
	// Checksum (overflowed sum of data)
	uint8_t sum = 0;
	for (uint16_t i = 0; i < sizeof(tx_data); i++) {
		sum += tx_data[i];
	}
	tx_data[len + 4] = sum;

	// Transmit packet to radio

	HAL_UART_Transmit(dev->huart, tx_data, len + 5, RADIO_UART_TIMEOUT_MS);
}
