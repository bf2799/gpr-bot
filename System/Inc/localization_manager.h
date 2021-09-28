/*
 * localization_manager.h
 *
 * Gathers sensor data from encoders, GPS, and IMU if available
 * Tracks estimated robot pose, velocity, and uncertainty in those measurements
 * Updates estimated pose, velocity, and uncertainty with new sensor data using Kalman filtering
 */

#ifndef INC_LOCALIZATION_MANAGER_H_
#define INC_LOCALIZATION_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "encoder.h"
#include "gps.h"
#include "imu.h"

typedef enum {
	GPS = 0,
	ENCODER_LEFT,
	ENCODER_RIGHT,
	IMU,
	NUM_SENSORS
} localization_sensor_type_t;

typedef struct vector_3d {
	double ew;
	double ns;
	double ud;
} vector_3d;

typedef struct quaternion_t {
	double w;
	vector_3d vec;
} quaternion_t;

/**
 * All represented in three dimensions, {east-west, north-south, up-down}
 * Reference for EW and NS may change depending on application.
 * Reference for UD is initial robot altitude
 * Assume the robot travels within a small enough area that the curvature of the earth can be ignored
 */
typedef struct localization_estimate_t {
	vector_3d pos;
	vector_3d vel;
	quaternion_t heading;
	quaternion_t ang_vel;
} localization_estimate_t;

/**
 * @brief Initializes sensors internally
 */
void localization_manager_init();

/**
 * @brief Enable the given sensors
 * @param[in] sensor_type: Type of the sensor to enable
 * @param[in] b_enable: Whether to enable (true) or disable (false) sensor from collecting
 */
void localization_manager_sensor_enable(localization_sensor_type_t sensor_type, bool b_enable);

/**
 * @brief Updates robot pose and velocity estimates by getting new sensor data
 * @return Pointer to current localization estimate
 */
localization_estimate_t* localization_manager_update_estimates();

#ifdef __cplusplus
}
#endif

#endif /* INC_LOCALIZATION_MANAGER_H_ */
