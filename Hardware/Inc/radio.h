/*
 * radio.h
 * Product: XBee®-PRO 900HP
 * Interface: UART
 * Only TX is supported right now since there is no current need for RX
 */

#ifndef INC_RADIO_H_
#define INC_RADIO_H_

#include "stm32f7xx_hal.h"

typedef struct radio_t {
	UART_HandleTypeDef* huart;
} radio_t;

/**
 * @brief Initialize radio
 * @param[out] dev: Radio device to initialize
 * @param[in] huart: UART handle corresponding to radio
 */
void radio_init(radio_t* dev, UART_HandleTypeDef* huart);

/**
 * @brief Transmit data over the radio
 * @param[in] data: Data to transmit over radio
 * @param[in] len: Length of data in bytes
 */
void radio_transmit(radio_t* dev, uint8_t* data, uint16_t len);

#endif /* INC_RADIO_H_ */
