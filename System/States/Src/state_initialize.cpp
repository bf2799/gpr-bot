/*
 * state_initialize.c
 */

#include "state_initialize.h"

#include "area_search_manager.h"
#include "drive_manager.h"
#include "gpr_manager.h"
#include "sensor_manager.h"
#include "telemetry_manager.h"

void InitializeState::init() {
	// Initialize all the managers, which in turn initialize the hardware
	drive_manager_init();
	gpr_manager_init();
	sensor_manager_init();
	telemetry_manager_init();

	// TODO: Get current location from localization manager
	pose2d_t initial_pose = {0, 0, 0};
	area_search_manager_generate_area(SEARCH_AREA_WIDTH_M, SEARCH_AREA_LENGTH_M, NUM_PASSES, STOPS_PER_PASS, initial_pose);
}

end_status_t InitializeState::run() {
	return end_status_t::InitializationComplete;
}

void InitializeState::cleanup() {}
