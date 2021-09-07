/*
 * signal_receiver.c
 */

#include "signal_receiver.h"

/**
 * @brief Callback for when signal receiving completes
 * @param hadc: ADC handle that finished receiving
 *
 * Sets callback to NULL to indicate completeness.
 */
static void signal_receiver_complete(ADC_HandleTypeDef* hadc) {
	hadc->ConvCpltCallback = NULL;
}

void signal_receiver_init(signal_receiver_t* dev, ADC_HandleTypeDef* hadc) {
	// Check user inputs
	if (!dev || !hadc) {
		return;
	}

	dev->hadc = hadc;
	hadc->ConvCpltCallback = NULL; // Notify
}

void signal_receiver_start(signal_receiver_t* dev, uint32_t num_samples) {
	// Check user inputs
	if (!dev || num_samples > SIG_RECEIVER_MAX_DMA_SAMPLES) {
		return;
	}

	// Check conversion isn't currently in progress
	if (dev->hadc->ConvCpltCallback != NULL) {
		return;
	}

	HAL_ADC_RegisterCallback(dev->hadc, HAL_ADC_CONVERSION_COMPLETE_CB_ID, signal_receiver_complete);
	HAL_ADC_Start_DMA(dev->hadc, dev->adc_dma_stream, num_samples);
}

bool signal_receiver_is_complete(signal_receiver_t* dev) {
	// Check user inputs
	if (!dev) {
		return true;
	}

	return (!dev->hadc->ConvCpltCallback); // If NULL, sampling is complete
}
