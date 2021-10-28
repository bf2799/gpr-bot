/*
 * pid_controller.c
 */

#include "pid_controller.h"
#include "stm32f7xx_hal.h"

void pid_controller_set_pid(pid_controller_t* controller, double p, double i, double d) {
	controller->kp = p;
	controller->ki = i;
	controller->kd = d;
	pid_controller_reset(controller);
}

double pid_controller_run(pid_controller_t* controller, double setpoint, double pv) {
	// Find current run time
	uint32_t cur_time_ms = HAL_GetTick();
	double dt = (double) (cur_time_ms - controller->last_err_time_ms);

	// Find current error
	double error = setpoint - pv;

	// Calculate P, I, and D terms
	double p_term = controller->kp * error;
	controller->i_accum += (error + controller->last_err) / 2 * dt;
	double i_term = controller->ki * controller->i_accum;
	double d_term = controller->kd * (error - controller->last_err) / dt;

	// Set last error and last error time appropriately
	controller->last_err = error;
	controller->last_err_time_ms = cur_time_ms;

	// Add P, I, and D terms and return
	double controller_out = p_term + i_term + d_term;
	return controller_out;
}

void pid_controller_reset(pid_controller_t* controller) {
	controller->i_accum = 0;
	controller->last_err = 0;
	controller->last_err_time_ms = HAL_GetTick();
}
