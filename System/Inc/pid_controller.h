/*
 * pid_controller.h
 *
 * Implements a PID controller with single-variable setpoint, measurement
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include <stdint.h>

typedef struct pid_controller_t {
	double kp;
	double ki;
	double kd;
	double i_accum;
	double last_err;
	uint32_t last_err_time_ms;
} pid_controller_t;

/**
 * @brief Sets the P, I, and D values of the PID controller
 * @param controller: PID controller object
 * @param p: Proportional feedback constant
 * @param i: Integral feedback constant
 * @param d: Derivative feedback constant
 */
void pid_controller_set_pid(pid_controller_t* controller, double p, double i, double d);

/**
 * @brief Runs PID controller given the current setpoint and measurement
 * @param controller: PID controller object
 * @param setpoint: Target value for PID-controlled system
 * @param pv: Process variable value (measured value)
 * @return Value to add to setpoint in next control command to better reach setpoint
 */
double pid_controller_run(pid_controller_t* controller, double setpoint, double pv);

/**
 * @brief Resets error accumulator and last errors of the PID controller
 * @param controller: Controller to reset
 */
void pid_controller_reset(pid_controller_t* controller);


#endif /* INC_PID_CONTROLLER_H_ */
