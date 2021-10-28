/*
 * drive_manager.c
 */

#include "drive_manager.h"

#include <math.h>

#include "drive_constants.h"
#include "motor.h"
#include "peripheral_assigner.h"
#include "pid_controller.h"

#define FLOAT_ZERO_BOUNDARY 0.001

#define DEFAULT_KP_VEL_WHEEL_L 	0
#define DEFAULT_KI_VEL_WHEEL_L 	0
#define DEFAULT_KD_VEL_WHEEL_L 	0
#define DEFAULT_KP_VEL_WHEEL_R 	0
#define DEFAULT_KI_VEL_WHEEL_R 	0
#define DEFAULT_KD_VEL_WHEEL_R 	0
#define DEFAULT_KP_HEADING		0
#define DEFAULT_KI_HEADING		0
#define DEFAULT_KD_HEADING		0

static motor_t motor_l;
static motor_t motor_r;

static pid_controller_t pid_ctrl_vel_wheel_l;
static pid_controller_t pid_ctrl_vel_wheel_r;
static pid_controller_t pid_ctrl_heading;

static double setpoint_forward_vel_mps = 0;
static double setpoint_turn_vel_radps = 0;
static double setpoint_heading_rad = 0;
static bool use_next_heading_as_target = false; // Whether next incoming heading should be used to determine the target

void drive_manager_init() {
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

	// Initialize PID controllers
	pid_controller_set_pid(&pid_ctrl_vel_wheel_l, DEFAULT_KP_VEL_WHEEL_L, DEFAULT_KI_VEL_WHEEL_L, DEFAULT_KD_VEL_WHEEL_L);
	pid_controller_set_pid(&pid_ctrl_vel_wheel_r, DEFAULT_KP_VEL_WHEEL_R, DEFAULT_KI_VEL_WHEEL_R, DEFAULT_KD_VEL_WHEEL_R);
	pid_controller_set_pid(&pid_ctrl_heading, DEFAULT_KP_HEADING, DEFAULT_KI_HEADING, DEFAULT_KD_HEADING);
}

void drive_manager_disable() {
	// Set motor percentages to 0
	motor_set_percentage(&motor_l, 0);
	motor_set_percentage(&motor_r, 0);
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

/**
 * @brief Turns wheel velocity setpoints into motor percentages
 * @param[in] wheel_vel_l: Wheel velocity setpoint for the left side
 * @param[in] wheel_vel_r: Wheel velocity setpoint for the right side
 * @param[out] motor_percent_l: Feed-forward motor percent for the given left wheel velocity
 * @param[out] motor_percent_r: Feed-foward motor percent for the given right wheel velocity
 */
static void wheel_vels_to_motor_percents(double wheel_vel_l, double wheel_vel_r, double* motor_percent_l, double* motor_percent_r) {
	// V = sign(v) * (k|v| + V0) (Voltage percent = velocity direction * [voltage_velocity_slope * abs(velocity) + static voltage offset])
	*motor_percent_l = VOLTAGE_VELOCITY_SLOPE_LEFT * fabs(wheel_vel_l) + VOLTAGE_STATIC_OFFSET_LEFT;
	*motor_percent_l *= wheel_vel_l > 0 ? 1 : -1;

	*motor_percent_r = VOLTAGE_VELOCITY_SLOPE_RIGHT * fabs(wheel_vel_r) + VOLTAGE_STATIC_OFFSET_RIGHT;
	*motor_percent_r *= wheel_vel_r > 0 ? 1 : -1;
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
		pid_controller_reset(&pid_ctrl_heading);
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

	double internal_setpoint_forward_vel_mps = setpoint_forward_vel_mps;
	double internal_setpoint_turn_vel_radps = setpoint_turn_vel_radps;

	// If turn velocity setpoint is 0 (straight line), update turn velocity setpoint by running PID on heading setpoint
	if (fabs(setpoint_turn_vel_radps) < FLOAT_ZERO_BOUNDARY) {
		internal_setpoint_turn_vel_radps += pid_controller_run(&pid_ctrl_heading, setpoint_heading_rad, state->ang_yaw);
	}

	// Convert state setpoints to wheel velocity setpoints
	double wheel_l_vel_mps_setpoint = 0;
	double wheel_r_vel_mps_setpoint = 0;
	state_vel_to_wheel_vel(internal_setpoint_forward_vel_mps, internal_setpoint_turn_vel_radps, &wheel_l_vel_mps_setpoint, &wheel_r_vel_mps_setpoint);

	// Convert state estimates to wheel velocity setpoints
	double wheel_l_vel_mps_pv = 0;
	double wheel_r_vel_mps_pv = 0;
	state_vel_to_wheel_vel(state->vel, state->ang_vel_yaw, &wheel_l_vel_mps_pv, &wheel_r_vel_mps_pv);

	// Calculate control wheel velocity setpoints based on PID feedback
	double wheel_l_vel_mps_control_setpoint = 0;
	double wheel_r_vel_mps_control_setpoint = 0;
	wheel_l_vel_mps_setpoint += pid_controller_run(&pid_ctrl_vel_wheel_l, wheel_l_vel_mps_setpoint, wheel_l_vel_mps_pv);
	wheel_r_vel_mps_setpoint += pid_controller_run(&pid_ctrl_vel_wheel_r, wheel_r_vel_mps_setpoint, wheel_r_vel_mps_pv);

	// Convert control wheel velocity setpoints to motor percentages
	double motor_percent_l;
	double motor_percent_r;
	wheel_vels_to_motor_percents(wheel_l_vel_mps_control_setpoint, wheel_r_vel_mps_control_setpoint, &motor_percent_l, &motor_percent_r);

	// Set left and right motor percentages
	motor_set_percentage(&motor_l, motor_percent_l);
	motor_set_percentage(&motor_r, motor_percent_r);
}
