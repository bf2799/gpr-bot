/*
 * area_search_manager.h
 *
 * Creates an area for searching, consisting of pose destinations along the way.
 */

#ifndef INC_AREA_SEARCH_MANAGER_H_
#define INC_AREA_SEARCH_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "trajectory_manager.h"

/**
 * @brief Generates a rectangular area for the robot to search with a given number of passes and recording stops per pass.
 * Robot starts in the bottom right.
 * @param[in] width_m: Width of the rectangular area (bottom or top sides)
 * @param[in] length_m: Height of the rectangular area (left or right sides)
 * @param[in] num_passes: Number of passes to make across the area, assuming "there and back" is 2 passes
 * @param[in] stops_per_pass: Number of times to stop in each pass, excluding the ends
 * @param[in] start_pose: Starting pose for the area generation (where robot is facing when in bottom left corner)
 */
void area_search_manager_generate_area(double width_m, double length_m, int num_passes, int stops_per_pass, pose2d_t start_pose);

/**
 * @brief Returns the next pose the robot should stop at and tracks that the last pose was correctly hit
 * @param[out] line_complete: Whether next destination is to get to another pass
 * @return Pose of next location
 */
pose2d_t area_search_manager_retrieve_next_destination(bool* line_complete);

/**
 * @brief Returns whether whole area was searched
 */
bool area_search_manager_is_complete();

#ifdef __cplusplus
}
#endif

#endif /* INC_AREA_SEARCH_MANAGER_H_ */
