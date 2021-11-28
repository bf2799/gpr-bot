/*
 * planning_manager.c
 */

#include "planning_manager.h"

#include <math.h>

#include "drive_constants.h"

#define TRAJECTORY_STOP_BAND_POSITION	0.03 // Meters
#define TRAJECTORY_STOP_BAND_ANGLE		0.01 // Radians

typedef enum  {
	INITIAL_ROTATION,
	DRIVE,
	FINAL_ROTATION
} sub_trajectories_t;

typedef struct vel_pos_pair_t {
	double vel;
	double pos;
} vel_pos_pair_t;

static vel_pos_pair_t active_trajectory[3][4];
static pose2d_t sub_trajectory_start_pose[3];
static pose2d_t sub_trajectory_end_pose[3];
static int current_sub_trajectory = INITIAL_ROTATION;

/**
 * @brief Calculates the initial rotation sub-trajectory as pairs of (angular velocity, angle) points relative to sub-trajectory start pose.
 * @param[in] start_pose: Start pose of the whole trajectory
 * @param[in] end_pose: End pose of the whole trajectory
 * @param[in] speed_multiplier: How much the trajectory velocities should be limited on scale of 0-1
 */
static void calculate_trajectory_initial_rotation(pose2d_t start_pose, pose2d_t end_pose, double speed_multiplier) {

	// Calculate angle from start position to end position
	double ang_diff_rad = atan2(end_pose.y - start_pose.y, end_pose.x - start_pose.x);

	// Calculate angle from start pose to end position
	ang_diff_rad -= start_pose.theta;

	// Figure out if triangle or trapezoidal profile is required
	// Boundary is the triangle velocity vs. time profile that has a slope of MAX_ACCEL and just touches MAX_SPEED
	// Change in position is the integral under the velocity vs time profile
	double delta_position_boundary = (MAX_DRIVE_SPEED_MPS * speed_multiplier) * (MAX_DRIVE_SPEED_MPS * speed_multiplier / MAX_DRIVE_ACCEL_MPSPS);
	if (ang_diff_rad < 0){
		delta_position_boundary = -delta_position_boundary;
	}
	double delta_position_actual = ang_diff_rad * WHEEL_BASE_M / 2;

	// Triangular profile
	if (fabs(delta_position_actual) < fabs(delta_position_boundary)) {

		// Calculate the velocity at the halfway point (accel will be MAX_ACCEL)
		// delta_position_actual = peak_vel^2 / MAX_ACCEL
		// peak_vel = sqrt(abs(delta_position_actual) / MAX_ACCEL) * sign(delta_position_actual)
		double peak_vel = sqrt(fabs(delta_position_actual) / MAX_DRIVE_ACCEL_MPSPS);
		if (delta_position_actual < 0) {
			peak_vel = -peak_vel;
		}

		// Convert back to angular components
		double peak_vel_angular = peak_vel / (WHEEL_BASE_M / 2);
		double delta_position_actual_angular = delta_position_actual / (WHEEL_BASE_M / 2);

		vel_pos_pair_t pairs[4] = {
				{0, 0},
				{peak_vel_angular, delta_position_actual_angular / 2},
				{0, 0},
				{0, 0}
		};
		for (int i = 0; i < 4; i++) {
			active_trajectory[INITIAL_ROTATION][i] = pairs[i];
		}
	}

	// Trapezoidal profile
	else {
		vel_pos_pair_t pairs[4] = {
				{0, 0},
				{MAX_DRIVE_SPEED_MPS * speed_multiplier / (WHEEL_BASE_M / 2), delta_position_boundary / 2 / (WHEEL_BASE_M / 2)},
				{MAX_DRIVE_SPEED_MPS * speed_multiplier / (WHEEL_BASE_M / 2), ang_diff_rad - delta_position_boundary / 2 / (WHEEL_BASE_M / 2)},
				{0, 0}
		};
		for (int i = 0; i < 4; i++) {
			active_trajectory[INITIAL_ROTATION][i] = pairs[i];
		}
	}
}

/**
 * @brief Calculates the drive sub-trajectory as pairs of (velocity, position) points relative to sub-trajectory start pose.
 * @param[in] start_pose: Start pose of the whole trajectory
 * @param[in] end_pose: End pose of the whole trajectory
 * @param[in] speed_multiplier: How much the trajectory velocities should be limited on scale of 0-1
 */
static void calculate_trajectory_drive(pose2d_t start_pose, pose2d_t end_pose, double speed_multiplier) {

	// Find distance between start pose and end pose
	double delta_position_actual = hypot(end_pose.y - start_pose.y, end_pose.x - start_pose.x);

	// Figure out if triangular or trapezoidal profile is required
	double delta_position_boundary = (MAX_DRIVE_SPEED_MPS * speed_multiplier) * (MAX_DRIVE_SPEED_MPS * speed_multiplier / MAX_DRIVE_ACCEL_MPSPS);

	// Triangular profile
	if (fabs(delta_position_actual) < fabs(delta_position_boundary)) {

		// Calculate the velocity at the halfway point (accel will be MAX_ACCEL)
		// delta_position_actual = peak_vel^2 / MAX_ACCEL
		// peak_vel = sqrt(abs(delta_position_actual) / MAX_ACCEL) * sign(delta_position_actual)
		double peak_vel = sqrt(fabs(delta_position_actual) / MAX_DRIVE_ACCEL_MPSPS);

		// Set velocity/position pairs
		vel_pos_pair_t pairs[4] = {
				{0, 0},
				{peak_vel, delta_position_actual / 2},
				{0, 0},
				{0, 0}
		};
		for (int i = 0; i < 4; i++) {
			active_trajectory[DRIVE][i] = pairs[i];
		}
	}

	// Trapezoidal profile
	else {
		vel_pos_pair_t pairs[4] = {
				{0, 0},
				{MAX_DRIVE_SPEED_MPS * speed_multiplier, delta_position_boundary / 2},
				{MAX_DRIVE_SPEED_MPS * speed_multiplier, delta_position_actual - delta_position_boundary / 2},
				{0, 0}
		};
		for (int i = 0; i < 4; i++) {
			active_trajectory[DRIVE][i] = pairs[i];
		}
	}

}

/**
 * @brief Calculates the final rotation sub-trajectory as pairs of (angular velocity, angle) points relative to sub-trajectory start pose.
 * @param[in] start_pose: Start pose of the whole trajectory
 * @param[in] end_pose: End pose of the whole trajectory
 * @param[in] speed_multiplier: How much the trajectory velocities should be limited on scale of 0-1
 */
static void calculate_trajectory_final_rotation(pose2d_t start_pose, pose2d_t end_pose, double speed_multiplier) {

	// Calculate angle to turn by subtracting angle of driving from end pose angle
	double ang_diff_rad = end_pose.theta - atan2(end_pose.y - start_pose.y, end_pose.x - start_pose.x);

	// Figure out if triangle or trapezoidal profile is required
	// Boundary is the triangle velocity vs. time profile that has a slope of MAX_ACCEL and just touches MAX_SPEED
	// Change in position is the integral under the velocity vs time profile
	double delta_position_boundary = (MAX_DRIVE_SPEED_MPS * speed_multiplier) * (MAX_DRIVE_SPEED_MPS * speed_multiplier / MAX_DRIVE_ACCEL_MPSPS);
	if (ang_diff_rad < 0){
		delta_position_boundary = -delta_position_boundary;
	}
	double delta_position_actual = ang_diff_rad * WHEEL_BASE_M / 2;

	// Triangular profile
	if (fabs(delta_position_actual) < fabs(delta_position_boundary)) {

		// Calculate the velocity at the halfway point (accel will be MAX_ACCEL)
		// delta_position_actual = peak_vel^2 / MAX_ACCEL
		// peak_vel = sqrt(abs(delta_position_actual) / MAX_ACCEL) * sign(delta_position_actual)
		double peak_vel = sqrt(fabs(delta_position_actual) / MAX_DRIVE_ACCEL_MPSPS);
		if (delta_position_actual < 0) {
			peak_vel = -peak_vel;
		}

		// Convert back to angular components
		double peak_vel_angular = peak_vel / (WHEEL_BASE_M / 2);
		double delta_position_actual_angular = delta_position_actual / (WHEEL_BASE_M / 2);

		vel_pos_pair_t pairs[4] = {
				{0, 0},
				{peak_vel_angular, delta_position_actual_angular / 2},
				{0, 0},
				{0, 0}
		};
		for (int i = 0; i < 4; i++) {
			active_trajectory[FINAL_ROTATION][i] = pairs[i];
		}
	}

	// Trapezoidal profile
	else {
		vel_pos_pair_t pairs[4] = {
				{0, 0},
				{MAX_DRIVE_SPEED_MPS * speed_multiplier / (WHEEL_BASE_M / 2), delta_position_boundary / 2 / (WHEEL_BASE_M / 2)},
				{MAX_DRIVE_SPEED_MPS * speed_multiplier / (WHEEL_BASE_M / 2), ang_diff_rad - delta_position_boundary / 2 / (WHEEL_BASE_M / 2)},
				{0, 0}
		};
		for (int i = 0; i < 4; i++) {
			active_trajectory[FINAL_ROTATION][i] = pairs[i];
		}
	}
}

/**
 * @brief Calculates the start poses and end poses of each sub trajectory
 * @param[in] start_pose: Start pose of the whole trajectory
 * @param[in] end_pose: End pose of the whole trajectory
 */
static void calculate_sub_trajectory_reference_poses(pose2d_t start_pose, pose2d_t end_pose) {

	// Set sub-trajectory start poses for initial rotation and drive
	sub_trajectory_start_pose[INITIAL_ROTATION] = start_pose;
	sub_trajectory_start_pose[DRIVE] = start_pose;
	sub_trajectory_start_pose[DRIVE].theta = atan2(end_pose.y - start_pose.y, end_pose.x - start_pose.x);
	// Set sub-trajectory start pose for final rotation
	sub_trajectory_start_pose[FINAL_ROTATION].x = end_pose.x;
	sub_trajectory_start_pose[FINAL_ROTATION].y = end_pose.y;
	sub_trajectory_start_pose[FINAL_ROTATION].theta = sub_trajectory_start_pose[DRIVE].theta;

	// Set sub-trajectory end poses
	sub_trajectory_end_pose[INITIAL_ROTATION] = sub_trajectory_start_pose[DRIVE];
	sub_trajectory_end_pose[DRIVE] = sub_trajectory_start_pose[FINAL_ROTATION];
	sub_trajectory_end_pose[FINAL_ROTATION] = end_pose;
}

void planning_manager_calculate_trajectory(pose2d_t start_pose, pose2d_t end_pose, double speed_multiplier) {

	// Check and scale user inputs. Assume start pose and end pose angles have already been scaled to -PI to PI
	if (speed_multiplier > 1) {
		speed_multiplier = 1;
	}
	if (speed_multiplier < 0) {
		speed_multiplier = 0;
	}

	// Calculate three components of trajectories as the active trajectory
	calculate_trajectory_initial_rotation(start_pose, end_pose, speed_multiplier);
	calculate_trajectory_drive(start_pose, end_pose, speed_multiplier);
	calculate_trajectory_final_rotation(start_pose, end_pose, speed_multiplier);

	// Calculate sub-trajectory reference poses (start and end)
	calculate_sub_trajectory_reference_poses(start_pose, end_pose);

	// Reset the sub-trajectory
	current_sub_trajectory = INITIAL_ROTATION;
}

/**
 * @brief Interpolates the y of an x value given the ends of the line segment defining the linear interpolation
 * @param[in] x_begin: x value of first line segment point
 * @param[in] x_end: x value of last line segment point
 * @param[in] y_begin: y value of first line segment point
 * @param[in] y_end: y value of last line segment point
 * @param[in] x_interp_val: x value to interpolate y of
 * @return Interpolated y value corresponding to given x
 */
static double interpolate(double x_begin, double x_end, double y_begin, double y_end, double x_interp_val) {
	return (y_end - y_begin) / (x_end - x_begin) * (x_interp_val - x_begin) + y_begin;
}

void planning_manager_follow_trajectory(pose2d_t cur_pose, double* forward_vel_mps, double* turn_vel_radps, bool* complete, bool* off_course) {

	// Set defaults for complete and off course
	*complete = false;
	*off_course = false;
	*forward_vel_mps = 0;
	*turn_vel_radps = 0;

	// Check if end condition for current sub-trajectory is met
	if (current_sub_trajectory == INITIAL_ROTATION || current_sub_trajectory == FINAL_ROTATION) {
		// If rotation, check if rotation is close to necessary rotation. Incorrect position will be caught and handled later
		if (fabs(sub_trajectory_end_pose[current_sub_trajectory].theta - cur_pose.theta) < TRAJECTORY_STOP_BAND_ANGLE) {
			if (current_sub_trajectory == FINAL_ROTATION) {
				*complete = true;
			}
			current_sub_trajectory += 1;
			return;
		}
	}
	else if (current_sub_trajectory == DRIVE) {
		// If driving, check if position is close to necessary position. Incorrect rotation will be caught and handled later
		if (hypot(sub_trajectory_end_pose[current_sub_trajectory].y - cur_pose.y, sub_trajectory_end_pose[current_sub_trajectory].x - cur_pose.x) < TRAJECTORY_STOP_BAND_POSITION) {
			current_sub_trajectory += 1;
			return;
		}
	}
	else {
		// Something is very wrong. We must be off track
		*off_course = true;
		return;
	}

	// Check whether robot is off-track. If so, abort and notify higher level
	// TODO: If off track, go into off-track mode to drive back to closest point on the sub-trajectory.
	if (current_sub_trajectory == INITIAL_ROTATION || current_sub_trajectory == FINAL_ROTATION) {
		// Check if position is off by more than double stop band amount
		if (hypot(sub_trajectory_start_pose[current_sub_trajectory].y - cur_pose.y, sub_trajectory_start_pose[current_sub_trajectory].x - cur_pose.x) > 2 * TRAJECTORY_STOP_BAND_POSITION) {
			*off_course = true;
			return;
		}
	}
	else {
		// Check if rotation is off by more than double stop band amount
		if (fabs(sub_trajectory_start_pose[current_sub_trajectory].theta - cur_pose.theta) > 2 * TRAJECTORY_STOP_BAND_ANGLE) {
			*off_course = true;
			return;
		}
	}

	// From sub-trajectory point closest to current pose, find velocity setpoints
	bool trajectory_setpoint_found = false;
	if (current_sub_trajectory == INITIAL_ROTATION || current_sub_trajectory == FINAL_ROTATION) {
		// Find current angle on trajectory
		for (int i = 0; i < 3; i++) {
			if ((sub_trajectory_start_pose[current_sub_trajectory].theta
					+ active_trajectory[current_sub_trajectory][i].pos
					< cur_pose.theta
					&& sub_trajectory_start_pose[current_sub_trajectory].theta
					+ active_trajectory[current_sub_trajectory][i + 1].pos
					< cur_pose.theta)
					|| (sub_trajectory_start_pose[current_sub_trajectory].theta
							+ active_trajectory[current_sub_trajectory][i].pos
							> cur_pose.theta
							&& sub_trajectory_start_pose[current_sub_trajectory].theta
							+ active_trajectory[current_sub_trajectory][i + 1].pos
							> cur_pose.theta)) {
				// Interpolate to find velocities
				*turn_vel_radps = interpolate(
						active_trajectory[current_sub_trajectory][i].pos,
						active_trajectory[current_sub_trajectory][i + 1].pos,
						active_trajectory[current_sub_trajectory][i].vel,
						active_trajectory[current_sub_trajectory][i + 1].vel,
						cur_pose.theta - sub_trajectory_start_pose[current_sub_trajectory].theta
				);
				trajectory_setpoint_found = true;
				break;
			}
		}
	}
	else {
		// Find closest position on trajectory to current position
		// Rotate current position around origin so trajectory is on the x-axis and find the x value to get current position on the trajectory
		double cur_relative_pos_x = cur_pose.x - sub_trajectory_start_pose[current_sub_trajectory].x;
		double cur_relative_pos_y = cur_pose.y - sub_trajectory_start_pose[current_sub_trajectory].y;
		double trajectory_rotation = -atan2(sub_trajectory_end_pose[current_sub_trajectory].y - sub_trajectory_start_pose[current_sub_trajectory].y, sub_trajectory_end_pose[current_sub_trajectory].x - sub_trajectory_start_pose[current_sub_trajectory].x);
		double cur_trajectory_position = cur_relative_pos_x * cos(trajectory_rotation) - cur_relative_pos_y * sin(trajectory_rotation);
		// Find current position on trajectory
		for (int i = 0; i < 3; i++) {
			if (cur_trajectory_position > active_trajectory[current_sub_trajectory][i].pos && cur_trajectory_position < active_trajectory[current_sub_trajectory][i + 1].pos) {
				// Interpolate to find correct velocities
				*forward_vel_mps = interpolate(
						active_trajectory[current_sub_trajectory][i].pos,
						active_trajectory[current_sub_trajectory][i + 1].pos,
						active_trajectory[current_sub_trajectory][i].vel,
						active_trajectory[current_sub_trajectory][i + 1].vel,
						cur_trajectory_position
				);
				trajectory_setpoint_found = true;
				break;
			}
		}
	}

	// If the rotation/position of the robot is off the expected trajectory, notify that the robot is off-course
	if (!trajectory_setpoint_found) {
		*off_course = true;
		return;
	}
}
