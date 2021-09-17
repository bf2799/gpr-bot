/*
 * sensor_manager.h
 * Manages sensor updates in one location
 */

#ifndef INC_SENSOR_MANAGER_H_
#define INC_SENSOR_MANAGER_H_

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
} sensor_type_t;

typedef union sensor_data_t {
	encoder_data_t encoder_data;
	gps_data_t gps_data;
	imu_data_t imu_data;
} sensor_data_t;

typedef struct sensor_t {
	bool enabled;
	bool updated;
	sensor_data_t last_data;
} sensor_t;

/**
 * @brief Initializes sensors internally
 */
void sensor_manager_init();

/**
 * @brief Enable the given sensors
 * @param[in] sensor_type: Type of the sensor to enable
 * @param[in] b_enable: Whether to enable (true) or disable (false) sensor from collecting
 */
void sensor_enable(sensor_type_t sensor_type, bool b_enable);

/**
 * @brief Gobble data from all enabled sensors.
 *
 * Some sensors may not have data always available even if available
 */
void sensor_read_data();

/**
 * @brief Reads available sensor data of all enabled sensors
 * @param[in] sensor_type:Type of sensor to get data from
 * @return Pointer to sensor data that was read, or NULL if no data available
 *
 * Note that some sensors may not always have new data to update with even if enabled, which is reflected in the "updated" field
 */
sensor_data_t* sensor_get_data(sensor_type_t sensor_type);

#endif /* INC_SENSOR_MANAGER_H_ */
