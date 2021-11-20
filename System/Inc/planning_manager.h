/*
 * planning_manager.h
 *
 * Holds an active 2D trajectory for the robot to follow in the global x-y plane from its starting pose.
 * All trajectories consist of three parts: an in-place rotation phase, a drive phase, and another in-place rotation phase.
 * Trajectories can be generated here by providing a final pose and speed multiplier assuming the initial is at position (0, 0) and rotation 0.
 *
 * Even though the robot may know its pose in 3D, the reference pose should be the robot's position and rotation as if it's in the 2D global x-y frame
 * This "flattening" allows the robot to plan through traverse hilly/bumpy terrain without knowing what the terrain is beforehand.
 *
 * A trajectory can be followed, outputting setpoint velocity and angular velocity setpoints.
 * When the final pose of a trajectory is "close enough," the planning manager's check for completion will be set true.
 *
 * If the trajectory ever gets off track, it will try to return the robot to the last spot on the trajectory that was reached.
 * This off-trajectory behavior was chosen because traversing the full straight line generated is important for mapping.
 * If the trajectory is off-track, the planning manager will be able to tell the user.
 *
 * A trajectory consists of a set of (velocity, position) or (angular velocity, rotation) pairs.
 * Following the trajectory will produce a set of (velocity, angular velocity) setpoints based on current pose.
 * Feeding back on pose instead of time prevents the feedback controller from making the setpoints unreasonably high if the robot is having trouble moving.
 * The generated trajectory will respect all dynamics of the robot, most importantly acceleration. This makes sticking to the setpoint much more manageable.
 *
 * When the trajectory does get off track, it will not just generate another trajectory to return to the correct position.
 * Since remaining in a straight line is no longer as important, the robot can simultaneously feedback on heading to the target and distance from the target.
 * Once it reaches target position, it can rotate to the correct heading.
 */

#ifndef INC_PLANNING_MANAGER_H_
#define INC_PLANNING_MANAGER_H_

#include <stdbool.h>

typedef struct pose2d_t {
	double x;
	double y;
	double theta;
} pose2d_t;

/**
 * @brief Calculates a new trajectory for the robot, canceling any previous one
 * @param[in] start_pose: Pose that the robot starts in
 * @param[in] end_pose: Pose for the robot to finish in
 * @param[in] speed_multiplier: 0-1 value to scale maximum planned speed of robot. Lower values will give more head room for control.
 */
void planning_manager_calculate_trajectory(pose2d_t start_pose, pose2d_t end_pose, double speed_multiplier);

/**
 * @brief Follow the set trajectory given a current robot pose in the "flattened" x-y global frame
 * @param[in] cur_pose: Current 2D pose of the robot
 * @param[out] forward_vel_mps: Setpoint for the robot's linear velocity in its own frame
 * @param[out] turn_vel_mps: Setpoint for the robot's angular velocity in its own frame
 * @param[out] complete: Whether trajectory is complete (true) or not (false)
 * @param[out] off_course: Whether robot is currently off-course from the trajectory (true) or on-course (false)
 */
void planning_manager_follow_trajectory(pose2d_t cur_pose, double* forward_vel_mps, double* turn_vel_radps, bool* complete, bool* off_course);

#endif /* INC_PLANNING_MANAGER_H_ */
