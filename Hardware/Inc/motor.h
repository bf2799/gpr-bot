/*
 * motor.h
 * Driver for Pololu TB9051FTG motor driver.
 * Purpose is to set motor percentage or voltage to a specific value
 * Currently, only brake stop mode supported (not coast)
 *
 * Note: Maximum PWM output of 20kHz, both top and bottom must be on for 10uS
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdbool.h>
#include "stm32f7xx_hal.h"

typedef struct motor_t {
	GPIO_TypeDef *en_port;
	uint16_t en_pin;
	GPIO_TypeDef *enb_port;
	uint16_t enb_pin;
	TIM_HandleTypeDef *htim_pwm1;
	uint32_t tim_channel_pwm1;
	TIM_HandleTypeDef *htim_pwm2;
	uint32_t tim_channel_pwm2;
	bool last_forward_drive;
	bool last_reverse_drive;
} motor_t;

/**
 * @brief Initialize motor hardware
 * @param[out] motor: Motor object to initialize
 * @param[in] en_port: Enable input GPIO port
 * @param[in] en_pin: Enable input GPIO pin
 * @param[in] enb_port: Inverted enable GPIO port
 * @param[in] enb_pin: Inverted enable GPIO pin
 * @param[in] htim_pwm1: Timer handle correlating to PWM1 output pin
 * @param[in] tim_channel_pwm1: Timer channel correlating to PWM1 output pin
 * @param[in] htim_pwm2: Timer handle correlating to PWM2 output pin
 * @param[in] tim_channel_pwm2: Tiemr channel correlating to PWM2 output pin
 */
void motor_init(
		motor_t *motor,
		GPIO_TypeDef *en_port, uint16_t en_pin,
		GPIO_TypeDef *enb_port, uint16_t enb_pin,
		TIM_HandleTypeDef *htim_pwm1, uint32_t tim_channel_pwm1,
		TIM_HandleTypeDef *htim_pwm2, uint32_t tim_channel_pwm2
);

/**
 * @brief Set motor percentage using duty cycle, regardless of voltage input
 * @param[in] motor: Motor to set percentage of
 * @param[in] pct: Percentage [-1 - 1]
 */
void motor_set_percentage(motor_t* motor, double pct);

#endif /* INC_MOTOR_H_ */
