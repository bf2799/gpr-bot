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

	dev->num_samples = num_samples;

	HAL_ADC_RegisterCallback(dev->hadc, HAL_ADC_CONVERSION_COMPLETE_CB_ID, signal_receiver_complete);
	HAL_ADC_Start_DMA(dev->hadc, dev->adc_dma_stream, num_samples);
}

uint32_t* signal_receiver_get_data(signal_receiver_t* dev, uint32_t* num_samples) {
	// Check user inputs
	if (!dev) {
		return NULL;
	}
	// If callback exists, it hasn't been called yet
	if (dev->hadc->ConvCpltCallback) {
		return NULL;
	}

	*num_samples = dev->num_samples;
	return dev->adc_dma_stream;
}
