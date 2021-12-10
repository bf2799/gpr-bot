/*
 * state_drive.cpp
 */

#include "state_drive.h"

#include "area_search_manager.h"
#include "drive_manager.h"
#include "localization_manager.h"
#include "trajectory_manager.h"

void DriveState::init() {
	// Check if area searching has completed. Stop everything if so
	if (area_search_manager_is_complete()) {
		search_complete_ = true;
		return;
	}
	// Get the next destination
	pose2d_t next_pose = area_search_manager_retrieve_next_destination(&line_complete_);
	// Get current pose from localization manager
	localization_manager_update_estimates();
	pose2d_t cur_pose = localization_manager_estimate_to_pose2d();
	// Calculate a trajectory to reach the next destination
	trajectory_manager_calculate_trajectory(cur_pose, next_pose, 1.0);
}

end_status_t DriveState::run() {

	// Disable if area search has completed
	if (search_complete_) {
		return end_status_t::SystemDisabled;
	}

	// Get current pose from localization manager
	localization_manager_update_estimates();
	pose2d_t cur_pose = localization_manager_estimate_to_pose2d();

	// Get next setpoints by following the trajectory
	double forward_vel_mps_setpoint;
	double turn_vel_radps_setpoint;
	bool trajectory_complete;
	bool trajectory_off_course;
	trajectory_manager_follow_trajectory(cur_pose, &forward_vel_mps_setpoint, &turn_vel_radps_setpoint, &trajectory_complete, &trajectory_off_course);

	// Apply setpoints from trajectory following to drive
	drive_state_estimation_t state_estimation = {};
	drive_manager_change_setpoint(forward_vel_mps_setpoint, turn_vel_radps_setpoint);
	drive_manager_run(&state_estimation);

	// Check if trajectory complete
	if (trajectory_complete) {
		return end_status_t::TrajectoryComplete;
	}

	//////// DEMO ONLY ////////
	// drive_manager_run_demo();

	return end_status_t::NoChange;
}

void DriveState::cleanup() {

}
