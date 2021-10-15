/*
 * motor.c
 */

#include "motor.h"
#include <math.h>

#define MOTOR_ZERO_EPSILON_PERCENT	0.00001

void motor_init(
		motor_t *motor,
		GPIO_TypeDef *en_port, uint16_t en_pin,
		GPIO_TypeDef *enb_port, uint16_t enb_pin,
		TIM_HandleTypeDef *htim_pwm1, uint32_t tim_channel_pwm1,
		TIM_HandleTypeDef *htim_pwm2, uint32_t tim_channel_pwm2) {

	// Check applicable user inputs
	if (!motor || !en_port || !enb_port || !htim_pwm1 || !htim_pwm2) {
		return;
	}

	// Set motor struct properties
	motor->en_port = en_port;
	motor->en_pin = en_pin;
	motor->enb_port = enb_port;
	motor->enb_pin = enb_pin;
	motor->htim_pwm1 = htim_pwm1;
	motor->tim_channel_pwm1 = tim_channel_pwm1;
	motor->htim_pwm2 = htim_pwm2;
	motor->tim_channel_pwm2 = tim_channel_pwm2;
	motor->last_forward_drive = false;
	motor->last_reverse_drive = false;

	// Set en and enb pins to support brake mode only. This is only need for driver at moment
	HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->enb_port, motor->enb_pin, GPIO_PIN_RESET);

	// Set motor percentage to 0
	motor_set_percentage(motor, 0);

	// Start PWM
	HAL_TIM_PWM_Start(motor->htim_pwm1, motor->tim_channel_pwm1);
	HAL_TIM_PWM_Start(motor->htim_pwm2, motor->tim_channel_pwm2);
}

void motor_set_percentage(motor_t* motor, double pct) {

	// If user input wrong, brake. Something is not right
	if (pct < -1 || pct > 1) {
		pct = 0;
	}

	uint32_t new_pulse = (uint32_t) (fabs(pct) * motor->htim_pwm1->Init.Period);

	// Forward: PWM1 used with PWM2 off if last command wasn't reverse
	if (pct > MOTOR_ZERO_EPSILON_PERCENT && !motor->last_reverse_drive) {
		__HAL_TIM_SET_COMPARE(motor->htim_pwm1, motor->tim_channel_pwm1, new_pulse);
		__HAL_TIM_SET_COMPARE(motor->htim_pwm2, motor->tim_channel_pwm2, 0);
		motor->last_forward_drive = true;
		motor->last_reverse_drive = false;
	}

	// Reverse: PWM2 used with PWM1 off if last command wasn't forward
	else if (pct < -MOTOR_ZERO_EPSILON_PERCENT && !motor->last_forward_drive) {
		__HAL_TIM_SET_COMPARE(motor->htim_pwm1, motor->tim_channel_pwm1, 0);
		__HAL_TIM_SET_COMPARE(motor->htim_pwm2, motor->tim_channel_pwm2, -new_pulse);
		motor->last_reverse_drive = true;
		motor->last_forward_drive = false;
	}

	// Brake: When pct low or attempting to go from forward to/from reverse to prevent breaking IC
	else {
		__HAL_TIM_SET_COMPARE(motor->htim_pwm1, motor->tim_channel_pwm1, 0);
		__HAL_TIM_SET_COMPARE(motor->htim_pwm2, motor->tim_channel_pwm2, 0);
		motor->last_reverse_drive = false;
		motor->last_forward_drive = false;
	}
}
