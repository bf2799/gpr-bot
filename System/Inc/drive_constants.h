/*
 * drive_constants.h
 *
 * Keeps track of kinematic-related robot properties in one place
 */

#ifndef INC_DRIVE_CONSTANTS_H_
#define INC_DRIVE_CONSTANTS_H_

#define MAX_DRIVE_SPEED_MPS				1   // Maximum speed of the robot in m/s when driving both sides at full power
#define MAX_DRIVE_ACCEL_MPSPS			1	// Maximum acceleration of robot in m/s^2 (used for planning, so may not be true dynamics)
#define WHEEL_BASE_M					0.2 // Wheel base of the robot in meters

#define VOLTAGE_VELOCITY_SLOPE_LEFT		0
#define VOLTAGE_VELOCITY_SLOPE_RIGHT	0
#define VOLTAGE_STATIC_OFFSET_LEFT		0
#define VOLTAGE_STATIC_OFFSET_RIGHT		0

#endif /* INC_DRIVE_CONSTANTS_H_ */
