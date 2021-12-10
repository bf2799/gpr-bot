/*
 * button.h
 *
 * Driver for reading from a button
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include <stdbool.h>

#include "stm32f7xx_hal.h"

typedef struct button_t {
	GPIO_TypeDef* button_port;
	uint16_t button_pin;
	uint32_t last_press_time_ms;
} button_t;

/**
 * @brief Initialize button hardware
 * @param[in, out] dev: Button device
 * @param[in] button_port: GPIO port for the button
 * @param[in] button_pin: GPIO pin for the button
 */
void button_init(button_t* dev, GPIO_TypeDef* button_port, uint16_t button_pin);

/**
 * @brief Return whether the button is currently pressed or not
 * @param[in] dev: Button device
 */
bool button_is_pressed(button_t* dev);

#endif /* INC_BUTTON_H_ */
