/*
 * encoder.h
 * Product: AMT10
 * Interface: Quadrature A/B inputs w/ 1 index pulse per revolution
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32f7xx_hal.h"

typedef struct encoder_t {
	TIM_HandleTypeDef* htim;
	int32_t tick_65536s;
	uint32_t tick_1s;
} encoder_t;

/**
 * @brief Initializes encoder device
 * @param[out] dev: Encoder device to initialize
 * @param[in] htim: Timer handle of encoder inputs
 */
void encoder_init(encoder_t* dev, TIM_HandleTypeDef* htim);

/**
 * @brief Start counting inputs from the encoder
 * @param[in] dev: Encoder device
 */
void encoder_start(const encoder_t* dev);

/**
 * @brief Stop counting inputs from the encoder
 * @param[in] dev: Encoder device
 */
void encoder_stop(const encoder_t* dev);

/**
 * @brief Get the current number of ticks for the encoder.
 * @param[in] dev: Encoder device
 * @param[out] tick_65536s: Ticks in the 65536s place
 * @param[out] tick_1s: Ticks in the ones place
 *
 * Assume this is called fast enough that any wrap around in the counter is closer to the last measurement
 */
void encoder_get_ticks(encoder_t* dev, int32_t* tick_65536s, uint32_t* tick_1s);

/**
 * @brief Define the current encoder value as 0
 * @param[in] dev: Encoder device
 */
void encoder_zero(encoder_t* dev);

#endif /* INC_ENCODER_H_ */
