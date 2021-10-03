/*
 * control_manager.h
 *
 * References state field in plan to generate a state-specific action list
 * Runs all sub-managers indicated by action list, ensuring each is provided a perception vector and a plan
 */

#ifndef INC_CONTROL_MANAGER_H_
#define INC_CONTROL_MANAGER_H_

#include "planning_manager.h"

/**
 * @brief Runs a single loop of the control manager, acting based on plan and perception of current events
 * @param perception_vector: Last perception of the world
 * @param plan: Plan to follow
 */
void control_manager_run(perception_t* perception_vector, plan_t* plan);

#endif /* INC_CONTROL_MANAGER_H_ */
