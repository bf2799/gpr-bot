/*
 * drive_manager.c
 */

#include "drive_manager.h"

#include <math.h>

#include "drive_constants.h"
#include "motor.h"
#include "peripheral_assigner.h"

#define FLOAT_ZERO_BOUNDARY 0.001

static motor_t motor_l;
static motor_t motor_r;

static double setpoint_forward_vel_mps = 0;
static double setpoint_turn_vel_radps = 0;
static double setpoint_heading_rad = 0;
static bool use_next_heading_as_target = false; // Whether next incoming heading should be used to determine the target

void driver_manager_init() {
	// Initialize hardware
	motor_init(
			&motor_l,
			MOTOR_L_EN_GPIO_Port,
			MOTOR_L_EN_Pin,
			MOTOR_L_ENB_GPIO_Port,
			MOTOR_L_ENB_Pin,
			MOTOR_LEFT_TIMER,
			MOTOR_LEFT_PWM1_TIMER_CHANNEL,
			MOTOR_LEFT_TIMER,
			MOTOR_LEFT_PWM2_TIMER_CHANNEL
	);
	motor_init(
			&motor_r,
			MOTOR_R_EN_GPIO_Port,
			MOTOR_R_EN_Pin,
			MOTOR_R_ENB_GPIO_Port,
			MOTOR_R_ENB_Pin,
			MOTOR_RIGHT_TIMER,
			MOTOR_RIGHT_PWM1_TIMER_CHANNEL,
			MOTOR_RIGHT_TIMER,
			MOTOR_RIGHT_PWM2_TIMER_CHANNEL
	);
}

/**
 * @brief Given a forward and angular velocity, calculates the left and right wheel velocities necessary to attain those inputs
 * @param[in] forward_vel_mps: Target robot velocity in forward direction (m/s)
 * @param[in] turn_vel_radps: Target robot angular velocity (rad/s)
 * @param[out] left_vel_mps: Resulting speed of the left side wheel (m/s)
 * @param[out] right_vel_mps: Resulting speed of the right side wheel (m/s)
 */
static void state_vel_to_wheel_vel(double forward_vel_mps, double turn_vel_radps, double* left_vel_mps, double* right_vel_mps) {
	/*
	 * forward_vel = (r + l)/2
	 * turn_vel = (r - l)/w
	 * r = 2 * forward_vel - l
	 * l = forward_vel - w * turn_vel / 2
	 */
	*left_vel_mps = forward_vel_mps - WHEEL_BASE_M * turn_vel_radps / 2;
	*right_vel_mps = 2 * forward_vel_mps - *left_vel_mps;
}

void drive_manager_change_setpoint(double forward_vel_mps, double turn_vel_radps) {
	// Convert user inputs to find estimated wheel speeds.
	double init_left_wheel_vel_mps, init_right_wheel_speed_mps;
	state_vel_to_wheel_vel(forward_vel_mps, turn_vel_radps, &init_left_wheel_vel_mps, &init_right_wheel_speed_mps);

	// If setpoints not physically possible, honor curvature but at slower speed
	double new_setpoint_forward_vel_mps = 0;
	double new_setpoint_turn_vel_radps = 0;
	if (fabs(init_left_wheel_vel_mps) > MAX_DRIVE_SPEED_MPS || fabs(init_right_wheel_speed_mps) > MAX_DRIVE_SPEED_MPS) {
		double divide_ratio = fmax(fabs(init_left_wheel_vel_mps), fabs(init_right_wheel_speed_mps)) / MAX_DRIVE_SPEED_MPS;
		new_setpoint_forward_vel_mps = forward_vel_mps / divide_ratio;
		new_setpoint_turn_vel_radps = turn_vel_radps / divide_ratio;
	}
	// Change setpoints normally if within physically possible bounds
	else {
		new_setpoint_forward_vel_mps = forward_vel_mps;
		new_setpoint_turn_vel_radps = turn_vel_radps;
	}
	// Change heading setpoint if turn setpoint is 0 and wasn't before
	if (fabs(new_setpoint_turn_vel_radps) < FLOAT_ZERO_BOUNDARY && fabs(setpoint_turn_vel_radps) > FLOAT_ZERO_BOUNDARY) {
		use_next_heading_as_target = true;
	}
	setpoint_forward_vel_mps = new_setpoint_forward_vel_mps;
	setpoint_turn_vel_radps = new_setpoint_turn_vel_radps;
}

void drive_manager_run(drive_state_estimation_t* state) {
	// Update heading setpoint to current heading if needed
	if (use_next_heading_as_target) {
		setpoint_heading_rad = state->ang_yaw;
		use_next_heading_as_target = false;
	}

	// If turn velocity setpoint is 0 (straight line), update turn velocity setpoint by running PID on heading setpoint

	// Convert state setpoints to wheel velocity setpoints
	// Convert state estimates to wheel velocity setpoints

	// Calculate wheel velocity error

	// Calculate control wheel velocity setpoints based on PID feedback

	// Convert control wheel velocity setpoints to motor percentages

	// Set left and right motor percentages
}
