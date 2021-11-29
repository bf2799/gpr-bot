/*
 * state_initialize.c
 */

#include "state_initialize.h"

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
}

end_status_t InitializeState::run() {
	return end_status_t::InitializationComplete;
}

void InitializeState::cleanup() {}
