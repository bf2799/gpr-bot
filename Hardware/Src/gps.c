/*
 * gps.c
 */

#include "gps.h"
#include "string.h"

static void gps_rx_complete(UART_HandleTypeDef* huart, uint16_t size) {
	// Indicate RX is complete by unregistering this callback
	HAL_UART_UnRegisterRxEventCallback(huart);
	huart->RxXferSize = size; // Since dev object not reachable here, cheat by setting RxXferSize now that it's not needed
}

void gps_init(gps_t* dev, UART_HandleTypeDef* huart) {
	// Check user inputs
	if (!dev || !huart) {
		return;
	}

	// Set initial dev properties
	dev->huart = huart;
	memset(dev->rx_data, 0, sizeof(dev->rx_data));

	// Assume GPS configurations have been saved on the GPS module
	// This way, code can remain the same when configurations change
	// Therefore, no need for UBX messages here
}

void gps_start_rx(gps_t* dev) {
	// Check user input
	if (!dev) {
		return;
	}

	// Register event callback and start DMA RX ended by IDLE line detection
	HAL_UART_RegisterRxEventCallback(dev->huart, gps_rx_complete);
	HAL_UARTEx_ReceiveToIdle_DMA(dev->huart, dev->rx_data, RX_DATA_SIZE);
}

static bool gps_set_last_data(gps_t* dev) {
	struct minmea_sentence_gga gga;
	enum minmea_sentence_id id = minmea_sentence_id((char*)(dev->rx_data), false);
	switch(id) {
	// Just process GGA data, no other messages needed
	case MINMEA_SENTENCE_GGA:
		if (minmea_parse_gga(&gga, (char*)(dev->rx_data))) {
			dev->last_data.gga = gga;
			return true;
		}
		break;
	default:
		break;
	}
	return false;
}

gps_data_t* gps_check_for_update(gps_t* dev) {
	// Check user input
	if (!dev) {
		return NULL;
	}

	if (dev->huart->RxEventCallback) {
		return NULL;
	}

	// Set last data with GPS processing
	gps_set_last_data(dev);
	return &dev->last_data;
}
