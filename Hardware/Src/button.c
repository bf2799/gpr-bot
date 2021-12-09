/*
 * button.c
 */

#include "button.h"

#define BUTTON_RESET_PERIOD_MS 500

void button_init(button_t* dev, GPIO_TypeDef* button_port, uint16_t button_pin) {
	// Check user input
	if (!dev || !button_port) {
		return;
	}

	dev->button_port = button_port;
	dev->button_pin = button_pin;
}

bool button_is_pressed(button_t* dev) {
	// Check user input
	if (!dev) {
		return false;
	}

	// If button is pressed and the reset period has passed since last press, button is pressed
	if ((HAL_GPIO_ReadPin(dev->button_port, dev->button_pin) == GPIO_PIN_SET) && (HAL_GetTick() - dev->last_press_time_ms > BUTTON_RESET_PERIOD_MS)) {
		dev->last_press_time_ms = HAL_GetTick();
		return true;
	}

	// If reached button is not pressed
	return false;
}
