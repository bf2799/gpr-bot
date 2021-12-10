/*
 * state_initialize.c
 */

#include "state_initialize.h"

#include "area_search_manager.h"
#include "drive_manager.h"
#include "gpr_manager.h"
#include "localization_manager.h"
#include "telemetry_manager.h"

void InitializeState::init() {
	// Initialize all the managers, which in turn initialize the hardware
	drive_manager_init();
	gpr_manager_init();
	localization_manager_init();
	telemetry_manager_init();

	// Enable sensors in localization manager
	localization_manager_sensor_enable(localization_sensor_type_t::ENCODER_LEFT, true);
	localization_manager_sensor_enable(localization_sensor_type_t::ENCODER_RIGHT, true);
	localization_manager_sensor_enable(localization_sensor_type_t::IMU, true);

	// Get current location from localization manager and generate search area
	localization_manager_update_estimates();
	pose2d_t initial_pose_2d = localization_manager_estimate_to_pose2d();
	area_search_manager_generate_area(SEARCH_AREA_WIDTH_M, SEARCH_AREA_LENGTH_M, NUM_PASSES, STOPS_PER_PASS, initial_pose_2d);
}

end_status_t InitializeState::run() {
	return end_status_t::InitializationComplete;
}

void InitializeState::cleanup() {}
