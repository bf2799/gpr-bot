/*
 * telemetry_manager.h
 *
 * Holds definition for telemetered packet protocol, including location, GPR frequency, and recorded GPR data in the body
 * Controls timing of radio to prevent oversending.
 */

#ifndef INC_TELEMETRY_MANAGER_H_
#define INC_TELEMETRY_MANAGER_H_

/**
 * @brief Initialize telemetry manager and its underlying hardware
 */
void telemetry_manager_init();

/**
 * @brief Telemeter relative robot pose to its initial position (when turned on)
 * @param pos_x: Estimated position in meters of the robot center relative to its starting position along its left-right axis (right positive)
 * @param pos_y: Estimated position in meters of the robot center relative to its starting position along its forward-backward axis (forward positive)
 * @param pos_z: Estimated position in meters of the robot center relative to its starting position along its up-down axis (up positive)
 * @param yaw: Estimated heading in degrees of the robot relative to its starting orientation in its x-y plane
 * @param roll: Estimated heading in degrees of the robot relative to its starting orientation in its x-z plane
 * @param pitch: Estimated heading in degrees of the robot relative to its starting orientation in its y-z plane
 */
void telemetry_manager_send_relative_pose(double pos_x, double pos_y, double pos_z, double yaw, double roll, double pitch);

/**
 * @brief Telemeter estimate of initial, absolute robot pose
 * @param longitude: Estimated starting position of the robot center (east-west)
 * @param latitude: Estimated starting position of the robot center (north-south)
 * @param elevation: Estimated starting elevation in meters of the robot center
 * @param yaw: Estimated starting heading in degrees of the robot in its longitude-latitude plane
 * @param roll: Estimated starting heading in degrees of the robot in its longitude-elevation plane
 * @param pitch: Estimated starting heading in degrees of the robot in its latitude-elevation plane
 */
void telemetry_manager_send_absolute_pose(double longitude, double latitude, double elevation, double yaw, double roll, double pitch);

/**
 * @brief Telemeter GPR data at a specific frequency
 * @param transmit_freq: Frequency in MHz of the signal that the GPR transmitter sent
 * @param mixer_ref_freq: Frequency in MHz of the reference signal the mixer was given to combine with the received signal
 * @param data_values: ADC inputs from the receiver
 * @param data_len: Number of samples in data
 */
void telemetry_manager_send_gpr_data(double transmit_freq, double mixer_ref_freq, uint32_t* data_values, uint16_t data_len);

/**
 * @brief Telemeter data that helps monitor the robot
 * @param battery_voltage: Voltage of battery
 */
void telemetry_manager_send_monitoring_data(double battery_voltage);\

bool telemetry_manager_ready_to_send();

#endif /* INC_TELEMETRY_MANAGER_H_ */
