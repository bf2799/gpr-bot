/*
 * encoder.c
 */

#include "encoder.h"

void encoder_init(encoder_t* dev, TIM_HandleTypeDef* htim) {
	// Check user input
	if (!dev || !htim) {
		return;
	}

	// Set initial encoder properties
	dev->htim = htim;

	// Zero encoder
	encoder_zero(dev);
}

void encoder_start(const encoder_t* dev) {
	// Check user input
	if (!dev) {
		return;
	}

	// Start recording encoders
	HAL_TIM_Encoder_Start(dev->htim, TIM_CHANNEL_ALL);
}

void encoder_stop(const encoder_t* dev) {
	// Check user input
	if (!dev) {
		return;
	}

	// Stop recording encoders
	HAL_TIM_Encoder_Stop(dev->htim, TIM_CHANNEL_ALL);
}

encoder_data_t* encoder_get_data(encoder_t* dev) {
	// Check user input
	if (!dev) {
		return NULL;
	}

	uint32_t cur_ticks = dev->htim->Instance->CNT;

	// If current ticks in lower third and last ticks in upper third, there was reverse wrap around
	if (cur_ticks < (dev->htim->Init.Period / 3) && dev->data.tick_1s > (dev->htim->Init.Period / 3 * 2)) {
		dev->data.tick_65536s -= 1;
	}
	else if (cur_ticks > (dev->htim->Init.Period / 3 * 2) && dev->data.tick_1s < (dev->htim->Init.Period / 3)) {
		dev->data.tick_65536s += 1;
	}

	dev->data.tick_1s = cur_ticks;
	return &(dev->data);
}

void encoder_zero(encoder_t* dev) {
	// Check user input
	if (!dev) {
		return;
	}

	// Set register counter and software counters to 0
	dev->htim->Instance->CNT = 0;
	dev->data.tick_1s = 0;
	dev->data.tick_65536s = 0;
}
