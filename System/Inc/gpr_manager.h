/*
 * gpr_manager.h
 * Manages all behavior related to ground penetrating radar
 */

#ifndef INC_GPR_MANAGER_H_
#define INC_GPR_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initializes the GPR hardware
 */
void gpr_manager_init();

/**
 * @brief Sends GPR signal pulse at given frequency and start recording results
 * @param[in] freq_mhz: Frequency to send out pulse at in MHz
 * @param[in] num_samples: Number of samples to record
 * @param[in] pulse_time_us: Time of pulse in microseconds
 */
void gpr_record_start(double freq_mhz, uint32_t num_samples, double pulse_time_us);

/**
 * @brief Gets pointer to data from the GPR if recording isn't in progress
 * @param[out] num_samples: Number of samples retrieved from the GPR
 * @return Pointer to data if recording not in progress, NULL if recording is in progress
 */
uint32_t* gpr_get_data(uint32_t* num_samples);

#endif /* INC_GPR_MANAGER_H_ */
