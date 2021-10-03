/*
 * planning_manager.h
 * Makes sure plans always run and are provided with perception vector.
 * No external hardware communication exists in the whole planning branch.
 * Conglomerates plans developed by all sub-managers into a single plan.
 */

#ifndef INC_PLANNING_MANAGER_H_
#define INC_PLANNING_MANAGER_H_

#include "perception_manager.h"

typedef struct plan_t {
	void* na;
} plan_t;

/**
 * @brief Finds new plan based on given perception vector
 * @param[in] perception_vector: Perception vector to base plan on
 * @return Resulting plan
 */
plan_t* planning_manager_run(const perception_t* perception_vector);


#endif /* INC_PLANNING_MANAGER_H_ */
