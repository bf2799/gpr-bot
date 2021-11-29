/*
 * gpr_manager.h
 * Manages all behavior related to ground penetrating radar
 */

#ifndef INC_GPR_MANAGER_H_
#define INC_GPR_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initializes the GPR hardware
 */
void gpr_manager_init();

/**
 * @brief Record GPR data in a step frequency sweep through the given frequency ranges
 * @param[in] start_freq_mhz: Frequency to start sweep at in MHz
 * @param[in] stop_freq_mhz: Frequency to stop sweep at
 * @param[in] num_steps: Number of steps in the frequency sweep
 * @param[in] num_samples_per_step: How long to record in each frequency step
 */
void gpr_manager_start_recording(double start_freq_mhz, double stop_freq, int num_steps, int num_samples_per_step);

/**
 * @brief Continue the GPR manager recording
 * @return True if GPR is still recording, false if GPR recording is complete
 */
bool gpr_manager_loop_recording();

/**
 * @brief Get data from GPR manager. Must be called while no recording is in progress to complete successfully.
 * @param[out] data: Start address of data, in 2D array of length (num_steps, array_samples_per_step)
 * @param[out] freqs_mhz: Frequencies (in MHz) matching up to each step in data
 * @param[out] actual_num_steps: Number of steps in data. Also, equals # of rows in data and length of freqs_mhz
 * @param[out] array_samples_per_step: Number of samples allocated in data array per step (length of row in data)
 * @param[out] actual_samples_per_step: Number of samples actually used in each data step (# of usable columns in data)
 * @return True if data retrieval was successful, False if failure (ie recording in progress)
 */
bool gpr_manager_get_data(uint32_t** data, double** freqs_mhz, int* actual_num_steps, int* array_samples_per_step, int* actual_samples_per_step);

#ifdef __cplusplus
}
#endif

#endif /* INC_GPR_MANAGER_H_ */
