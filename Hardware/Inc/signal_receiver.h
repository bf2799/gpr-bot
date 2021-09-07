/*
 * signal_receiver.h
 *
 * Driver for the ground-penetrating radar's signal receiver
 * Interface: ADC sampling
 */

#ifndef INC_SIGNAL_RECEIVER_H_
#define INC_SIGNAL_RECEIVER_H_

#include "stm32f7xx_hal.h"
#include <stdbool.h>

#define SIG_RECEIVER_MAX_DMA_SAMPLES 1000

typedef struct signal_receiver_t {
	ADC_HandleTypeDef* hadc;
	uint32_t adc_dma_stream[SIG_RECEIVER_MAX_DMA_SAMPLES];
} signal_receiver_t;

/**
 * @brief Initializes signal receiver. Must be called before any other functions
 * @param dev: Signal receiver device to initialize
 * @param hadc: ADC handle to receive from
 */
void signal_receiver_init(signal_receiver_t* dev, ADC_HandleTypeDef* hadc);

/**
 * @brief Record a number of bytes from the receiver via ADC
 * @param dev: Signal receiver device
 * @param num_samples: Number of samples to take from the receiver
 *
 * Uses DMA, so no additional CPU intervention is required
 */
void signal_receiver_start(signal_receiver_t* dev, uint32_t num_samples);

/**
 * @brief Check whether signal receiver has completed its sampling
 * @param dev: Signal receiver device
 * @return Whether sampling complete (true) or in progress (false)
 */
bool signal_receiver_is_complete(signal_receiver_t* dev);

#endif /* INC_SIGNAL_RECEIVER_H_ */
