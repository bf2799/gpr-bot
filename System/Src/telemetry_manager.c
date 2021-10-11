/*
 * telemetry_manager.c
 */

#include "radio.h"

#include "peripheral_assigner.h"

typedef enum {
	NA = 0,
	RelativePose,
	AbsolutePose,
	GPR,
	Monitoring
} message_id;

static struct message_header_t {
	uint8_t message_id;
	uint16_t payload_len;
} message_header;

static struct pose_payload_t {
	float pos_x;
	float pos_y;
	float pos_z;
	float yaw;
	float roll;
	float pitch;
} pose_payload;

static struct gpr_payload_t {
	float transmit_freq;
	float mixer_ref_freq;
} gpr_payload;

static struct monitoring_payload_t {
	float battery_voltage;
} monitoring_payload;

static radio_t radio;

void telemetry_manager_init() {
	radio_init(&radio, RADIO_UART);
}

void telemetry_manager_send_relative_pose(double pos_x, double pos_y, double pos_z, double yaw, double roll, double pitch) {
	// Set message header
	message_header.message_id = RelativePose;
	message_header.payload_len = sizeof(pose_payload);
	// Set message payload
	pose_payload.pos_x = (float) pos_x;
	pose_payload.pos_y = (float) pos_y;
	pose_payload.pos_z = (float) pos_z;
	pose_payload.yaw = (float) yaw;
	pose_payload.roll = (float) roll;
	pose_payload.pitch = (float) pitch;
	// Transmit message header
	radio_transmit(&radio, (uint8_t*) &message_header, sizeof(message_header));
	// Transmit message payload
	radio_transmit(&radio, (uint8_t*) &pose_payload, sizeof(pose_payload));
}

void telemetry_manager_send_absolute_pose(double longitude, double latitude, double elevation, double yaw, double roll, double pitch) {
	// Set message header
	message_header.message_id = AbsolutePose;
	message_header.payload_len = sizeof(pose_payload);
	// Set message payload
	pose_payload.pos_x = (float) longitude;
	pose_payload.pos_y = (float) latitude;
	pose_payload.pos_z = (float) elevation;
	pose_payload.yaw = (float) yaw;
	pose_payload.roll = (float) roll;
	pose_payload.pitch = (float) pitch;
	// Transmit message header
	radio_transmit(&radio, (uint8_t*) &message_header, sizeof(message_header));
	// Transmit message payload
	radio_transmit(&radio, (uint8_t*) &pose_payload, sizeof(pose_payload));
}

void telemetry_manager_send_gpr_data(double transmit_freq, double mixer_ref_freq, uint32_t* data_values, uint16_t data_len) {
	// Set message header
	message_header.message_id = GPR;
	message_header.payload_len = sizeof(gpr_payload) + data_len * sizeof(uint32_t); // 4 = size of uint32_t
	// Set message payload
	gpr_payload.transmit_freq = (float) transmit_freq;
	gpr_payload.mixer_ref_freq = (float) mixer_ref_freq;
	// Transmit message header
	radio_transmit(&radio, (uint8_t*) &message_header, sizeof(message_header));
	// Transmit message payload
	radio_transmit(&radio, (uint8_t*) &gpr_payload, sizeof(gpr_payload));
	// Transmit GPR data
	radio_transmit(&radio, (uint8_t*) &data_values, data_len * sizeof(uint32_t));
}

void telemetry_manager_send_monitoring_data(double battery_voltage) {
	// Set message header
	message_header.message_id = Monitoring;
	message_header.payload_len = sizeof(monitoring_payload);
	// Set message payload
	monitoring_payload.battery_voltage = (float) battery_voltage;
	// Transmit message header
	radio_transmit(&radio, (uint8_t*) &message_header, sizeof(message_header));
	// Transmit message payload
	radio_transmit(&radio, (uint8_t*) &monitoring_payload, sizeof(monitoring_payload));
}
