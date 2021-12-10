/*
 * drive_manager.h
 *
 * Scales user drive setpoints as needed to maintain physically attainable movement
 * Adjusts motor setpoints based on drive setpoints and current pose/velocity
 * Passes motor setpoints to motor controller driver
 */

#ifndef INC_DRIVE_MANAGER_H_
#define INC_DRIVE_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct drive_state_estimation_t {
	double vel;
	double ang_yaw;
	double ang_vel_yaw;
} drive_state_estimation_t;

/**
 * @brief Initializes drive manager hardware
 */
void drive_manager_init();

/**
 * @brief Disable drive manager by ensuring motors aren't getting any input
 */
void drive_manager_disable();

/**
 * @brief Change the velocity setpoint of the robot
 * @param[in] forward_vel_mps: Target velocity along the robot's y axis (forward-backward) in meters per second
 * @param[in] turn_vel_rad: Target angular velocity of the robot (counterclockwise positive) in radians per second
 */
void drive_manager_change_setpoint(double forward_vel_mps, double turn_vel_radps);

/**
 * @brief Runs a single instance of the drive control loop
 * @param[in] state: Subset of estimated robot state that drive manager needs to run feedback loop
 */
void drive_manager_run(drive_state_estimation_t* state);

/**
 * @brief Controls robot motor speed with button press
 */
void drive_manager_run_demo();

#ifdef __cplusplus
}
#endif

#endif /* INC_DRIVE_MANAGER_H_ */
