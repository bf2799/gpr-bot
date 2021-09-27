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

typedef union sensor_data_t {
	encoder_data_t encoder_data;
	gps_data_t gps_data;
	imu_data_t imu_data;
} localization_sensor_data_t;

typedef struct sensor_t {
	bool enabled;
	bool updated;
	localization_sensor_data_t last_data;
} localization_sensor_t;

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

#ifdef __cplusplus
}
#endif

#endif /* INC_LOCALIZATION_MANAGER_H_ */
