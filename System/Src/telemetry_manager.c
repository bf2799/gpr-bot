/*
 * telemetry_manager.c
 */

#include "telemetry_manager.h"

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
static int cur_transmit_queue_size;
static uint32_t last_transmit_time_ms;

static void telemetry_manager_update_queue_size() {
	cur_transmit_queue_size -= RADIO_TRANSMIT_SPEED_BPS * (HAL_GetTick() - last_transmit_time_ms) / 1000;
	if (cur_transmit_queue_size < 0) {
		cur_transmit_queue_size = 0;
	}
	last_transmit_time_ms = HAL_GetTick();
}

void telemetry_manager_init() {
	radio_init(&radio, RADIO_UART);

	uint8_t data[11] = "Hello there";
	while (1) {
		radio_transmit(&radio, data, 11);
	}
}

bool telemetry_manager_send_relative_pose(double pos_x, double pos_y, double pos_z, double yaw, double roll, double pitch) {
	// Update estimated queue size
	telemetry_manager_update_queue_size();

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

	// Check if we can transmit into queue
	uint16_t transmit_len = sizeof(message_header) + message_header.payload_len;
	if (cur_transmit_queue_size + transmit_len > RADIO_QUEUE_SIZE) {
		return false;
	}

	// Transmit message header
	radio_transmit(&radio, (uint8_t*) &message_header, sizeof(message_header));
	// Transmit message payload
	radio_transmit(&radio, (uint8_t*) &pose_payload, sizeof(pose_payload));
	cur_transmit_queue_size += transmit_len;

	return true;
}

bool telemetry_manager_send_absolute_pose(double longitude, double latitude, double elevation, double yaw, double roll, double pitch) {
	// Update estimated queue size
	telemetry_manager_update_queue_size();

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

	// Check if we can transmit into queue
	uint16_t transmit_len = sizeof(message_header) + message_header.payload_len;
	if (cur_transmit_queue_size + transmit_len > RADIO_QUEUE_SIZE) {
		return false;
	}

	// Transmit message header
	radio_transmit(&radio, (uint8_t*) &message_header, sizeof(message_header));
	// Transmit message payload
	radio_transmit(&radio, (uint8_t*) &pose_payload, sizeof(pose_payload));
	cur_transmit_queue_size += transmit_len;
	return true;
}

bool telemetry_manager_send_gpr_data(double transmit_freq, double mixer_ref_freq, uint32_t* data_values, uint16_t data_len, bool restart) {

	// Set up internal state for continuing/restarting long data transmissions
	static int next_data_byte_idx = 0;
	static bool header_sent = false;
	if (restart) {
		next_data_byte_idx = 0;
		header_sent = false;
	}

	// Update estimated queue size
	telemetry_manager_update_queue_size();

	if (!header_sent) {
		// Set message header
		message_header.message_id = GPR;
		message_header.payload_len = sizeof(gpr_payload) + data_len * sizeof(uint32_t); // 4 = size of uint32_t
		// Set message payload
		gpr_payload.transmit_freq = (float) transmit_freq;
		gpr_payload.mixer_ref_freq = (float) mixer_ref_freq;

		// Check if we can transmit header/beginning of payload into queue
		uint16_t transmit_len = sizeof(message_header) + sizeof(gpr_payload);
		if (cur_transmit_queue_size + transmit_len > RADIO_QUEUE_SIZE) {
			return false;
		}

		// Transmit message header
		radio_transmit(&radio, (uint8_t*) &message_header, sizeof(message_header));
		// Transmit message payload beginning
		radio_transmit(&radio, (uint8_t*) &gpr_payload, sizeof(gpr_payload));
		cur_transmit_queue_size += transmit_len;
		header_sent = true;
	}
	if (header_sent) {
		// Check how many bytes we can put into queue
		uint16_t transmit_len = RADIO_QUEUE_SIZE - cur_transmit_queue_size;
		bool should_stop = false;
		if (transmit_len + next_data_byte_idx > data_len * sizeof(uint32_t)) {
			transmit_len = data_len * sizeof(uint32_t) - next_data_byte_idx;
			should_stop = true;
		}

		// Transmit data
		radio_transmit(&radio, (uint8_t*) (data_values + next_data_byte_idx), transmit_len);
		cur_transmit_queue_size += transmit_len;
		next_data_byte_idx += transmit_len;
		return should_stop;
	}

	// Return here should only occur if heading wasn't sent
	return false;
}

bool telemetry_manager_send_monitoring_data(double battery_voltage) {
	// Update estimated queue size
	telemetry_manager_update_queue_size();

	// Set message header
	message_header.message_id = Monitoring;
	message_header.payload_len = sizeof(monitoring_payload);
	// Set message payload
	monitoring_payload.battery_voltage = (float) battery_voltage;

	// Check if we can transmit into queue
	uint16_t transmit_len = sizeof(message_header) + message_header.payload_len;
	if (cur_transmit_queue_size + transmit_len > RADIO_QUEUE_SIZE) {
		return false;
	}

	// Transmit message header
	radio_transmit(&radio, (uint8_t*) &message_header, sizeof(message_header));
	// Transmit message payload
	radio_transmit(&radio, (uint8_t*) &monitoring_payload, sizeof(monitoring_payload));
	cur_transmit_queue_size += transmit_len;
	return true;
}
