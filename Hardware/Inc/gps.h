/*
 * gps.h
 *
 * Driver for the GPS
 * Part: uBlox SAM-M8Q
 * Interface: UART
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f7xx_hal.h"
#include "minmea.h"

#define RX_DATA_SIZE 200

typedef struct gps_data_t {
	struct minmea_sentence_gga gga;
} gps_data_t;

typedef struct gps_t {
	UART_HandleTypeDef* huart;
	uint8_t rx_data[RX_DATA_SIZE];
	gps_data_t last_data;
} gps_t;

/**
 * @brief Initialize GPS
 * @param[out] dev: GPS device to initialize
 * @param[in] huart: UART handle for communication with GPS
 */
void gps_init(gps_t* dev, UART_HandleTypeDef* huart);

/**
 * @brief Start reception of one GPS packet
 * @param[in] dev: GPS device
 */
void gps_start_rx(gps_t* dev);

/**
 * @brief Checks if new packet has arrived since this function was last called. If so, returns its data
 * @param[in] dev: GPS device
 * @return NULL if there's no update, pointer to last data otherwise
 */
gps_data_t* gps_check_for_update(gps_t* dev);

#endif /* INC_GPS_H_ */
