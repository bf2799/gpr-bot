/*
 * scheduler.c
 */

#include "scheduler.h"

#include "control_manager.h"
#include "perception_manager.h"
#include "planning_manager.h"

void scheduler_run() {

	// Run the perception manager and receive a perception vector
	perception_t* perception_vector = perception_manager_run();

	// Run the planning manager and receive a plan
	plan_t* plan = planning_manager_run(perception_vector);

	// Run the control manager
	control_manager_run(perception_vector, plan);
}
