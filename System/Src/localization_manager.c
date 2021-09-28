/*
 * localization_manager.c
 */

#include "localization_manager.h"
#include "peripheral_assigner.h"

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

static localization_sensor_t sensors[NUM_SENSORS];

static encoder_t encoder_l;
static encoder_t encoder_r;
static gps_t gps;
static imu_t imu;
static localization_estimate_t cur_estimate;
static double cur_estimate_uncertainty[sizeof(localization_estimate_t)][sizeof(localization_estimate_t)]; //
static vector_3d pos_ref; // Reference position, may be 0 or updated by GPS

void localization_manager_init() {
	// Initialize sensors
	encoder_init(&encoder_l, ENCODER_LEFT_TIMER);
	encoder_init(&encoder_r, ENCODER_RIGHT_TIMER);
	gps_init(&gps, GPS_UART);
	imu_init(&imu, IMU_I2C);

	// Initialize sensor structs
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

/**
 * @brief Gobble data from all enabled sensors.
 *
 * Some sensors may not have data always available even if available
 */
static void localization_manager_read_sensor_data() {

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

localization_estimate_t* localization_manager_update_estimates() {

	// Predict current state based on last system input and last state

	// Read new sensor data
	localization_manager_read_sensor_data();

	// Update estimate based on new GPS data
	if (sensors[GPS].updated) {
		//sensors[GPS].last_data.gps_data;
	}

	// Update estimate based on new encoder data
	if (sensors[ENCODER_LEFT].updated && sensors[ENCODER_RIGHT].updated) {
		//sensors[ENCODER_LEFT].last_data.encoder_data;
		//sensors[ENCODER_RIGHT].last_data.encoder_data;
	}

	// Update estimate based on new IMU data
	if (sensors[IMU].updated) {
		//sensors[IMU].last_data.imu_data;
	}

	return &cur_estimate;
}
