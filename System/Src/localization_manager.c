/*
 * localization_manager.c
 */

#include "localization_manager.h"
#include "peripheral_assigner.h"

static localization_sensor_t sensors[NUM_SENSORS];

static encoder_t encoder_l;
static encoder_t encoder_r;
static gps_t gps;
static imu_t imu;

/**
 * @brief Gobble data from all enabled sensors.
 *
 * Some sensors may not have data always available even if available
 */
static void sensor_read_data() {

	for (int i = 0; i < NUM_SENSORS; i++) {
		sensors[i].updated = false;
	}

	if (sensors[ENCODER_LEFT].enabled) {
		// Encoder is asynchronous but with very high frequency update, so it should always have new data available
		sensors[ENCODER_LEFT].last_data.encoder_data = *encoder_get_data(&encoder_l);
		sensors[ENCODER_LEFT].updated = true;
	}

	if (sensors[ENCODER_RIGHT].enabled) {
		sensors[ENCODER_RIGHT].last_data.encoder_data = *encoder_get_data(&encoder_r);
		sensors[ENCODER_RIGHT].updated = true;
	}

	if (sensors[GPS].enabled) {
		gps_data_t* temp_gps_data = gps_check_for_update(&gps);
		// GPS is asynchronous but low frequency update, so it will not always have data available
		if (temp_gps_data) {
			sensors[GPS].last_data.gps_data = *temp_gps_data;
			sensors[GPS].updated = true;
			gps_start_rx(&gps);
		}
	}

	if (sensors[IMU].enabled) {
		// IMU is synchronous, so requesting data will return data
		sensors[IMU].last_data.imu_data = *imu_get_data(&imu);
		sensors[IMU].updated = true;
	}

}

/**
 * @brief Reads available sensor data of all enabled sensors
 * @param[in] sensor_type:Type of sensor to get data from
 * @return Pointer to sensor data that was read, or NULL if no data available
 *
 * Note that some sensors may not always have new data to update with even if enabled, which is reflected in the "updated" field
 */
static localization_sensor_data_t* sensor_get_data(localization_sensor_type_t sensor_type) {
	// Check user input
	if (sensor_type < 0 || sensor_type > NUM_SENSORS) {
		return NULL;
	}

	if (!sensors[sensor_type].updated) {
		return NULL;
	}

	return &(sensors[sensor_type].last_data);
}

void localization_manager_init() {
	// Initialize sensors
	encoder_init(&encoder_l, ENCODER_LEFT_TIMER);
	encoder_init(&encoder_r, ENCODER_RIGHT_TIMER);
	gps_init(&gps, GPS_UART);
	imu_init(&imu, IMU_I2C);

	// Initialize sensor manager sensor structs
	for (int i = 0; i < NUM_SENSORS; i++) {
		sensors[i].enabled = false;
		sensors[i].updated = false;
	}

	// Start asynchronous sensors
	encoder_start(&encoder_l);
	encoder_start(&encoder_r);
	gps_start_rx(&gps);
}

void localization_manager_sensor_enable(localization_sensor_type_t sensor_type, bool b_enable) {
	// Check user input
	if (sensor_type < 0 || sensor_type > NUM_SENSORS) {
		return;
	}

	sensors[sensor_type].enabled = b_enable;
}
