/*
 * gpr_manager.c
 */

#include "gpr_manager.h"

#include <string.h>

#include "peripheral_assigner.h"
#include "signal_generator.h"
#include "signal_receiver.h"
#include "stm32f7xx_hal.h"

#define ADC_SAMPLING_RATE_HZ 		1333333 + 1./3 // ADC_CLK / (Sampling Cycles + Resolution Cycles) = 24000000 / (3 + 15)
#define ADC_TARGET_INPUT_FREQ_HZ    ADC_SAMPLING_RATE_HZ / 2 * 0.9 // Nyquist frequency with 10% headroom to prevent aliasing
#define MAX_STEP_INCREMENTS			50
#define PULSE_TIME_US				1. // Pulse time in microseconds

static signal_generator_t sig_gen;
static signal_receiver_t sig_rec;
static signal_generator_t sig_rec_reference;

static uint32_t last_data[MAX_STEP_INCREMENTS][SIG_RECEIVER_MAX_DMA_SAMPLES]; // Most recent recorded data
static double last_frequencies[MAX_STEP_INCREMENTS]; // Most recent recorded frequencies
static double start_freq_mhz; // What frequency sweep was started at
static double stop_freq_mhz; // What frequency sweep was stopped at
static double freq_step_size_mhz; // How much to increment in each frequency sweep
static int num_samples_per_step; // How many samples are actually used in each frequency step
static int current_step_num; // What the current step number is in the sequence, starting at 0
static bool is_recording; // Whether GPR is currently in recording state

/**
 * @brief Callback for when GPR signal generation is complete.
 *
 * Stops signal generator
 */
static void gpr_manager_sig_gen_cplt(TIM_HandleTypeDef* htim) {
	(void) htim; // Unused, just needed for callback
	signal_generator_stop(&sig_gen);
	HAL_TIM_Base_Stop_IT(GPR_MANAGER_TIMER);
}

void gpr_manager_init() {
	// Initialize GPR hardware
	signal_generator_init(
			&sig_gen,
			SIGNAL_GENERATOR_SPI,
			SIG_GEN_LE_GPIO_Port,
			SIG_GEN_LE_Pin,
			SIGNAL_GENERATOR_TIMER,
			SIGNAL_GENERATOR_TIMER_CHANNEL,
			SystemCoreClock
	);
	signal_receiver_init(&sig_rec, SIGNAL_RECEIVER_ADC);
	signal_generator_init(
			&sig_rec_reference,
			SIGNAL_RECEIVER_REF_SPI,
			SIG_REC_REF_LE_GPIO_Port,
			SIG_REC_REF_LE_Pin,
			SIGNAL_RECEIVER_REF_TIMER,
			SIGNAL_RECEIVER_REF_TIMER_CHANNEL,
			SystemCoreClock
	);
}

/**
 * @brief Start transmitting/recording GPR at a specific frequency
 * @param[in] freq_mhz: Frequency to start recording at
 * @param[in] num_samples: Number of samples to record
 */
static void gpr_record_start(double freq_mhz, uint32_t num_samples) {
	// Set output frequency of signal generator
	signal_generator_set_output_freq(&sig_gen, freq_mhz);
	// Set reference frequency for signal receiver and start the reference clock
	signal_generator_set_output_freq(&sig_gen, freq_mhz - (ADC_TARGET_INPUT_FREQ_HZ / 1000000.));
	signal_generator_start(&sig_rec_reference);
	// Start recording on signal receiver
	signal_receiver_start(&sig_rec, num_samples);


	// Start signal generator and timer for stopping the signal generator
	uint32_t new_period = (uint32_t) ((double) SystemCoreClock * PULSE_TIME_US / 1000000.0);
	__HAL_TIM_SET_AUTORELOAD(GPR_MANAGER_TIMER, new_period);
	signal_generator_start(&sig_gen);
	HAL_TIM_Base_Start_IT(GPR_MANAGER_TIMER);
	HAL_TIM_RegisterCallback(GPR_MANAGER_TIMER, HAL_TIM_PERIOD_ELAPSED_CB_ID, gpr_manager_sig_gen_cplt);
}

void gpr_manager_start_recording(double start_freq_mhz_, double stop_freq_mhz_, int num_steps_, int num_samples_per_step_) {
	if (is_recording || num_steps_ > MAX_STEP_INCREMENTS || num_samples_per_step_ > SIG_RECEIVER_MAX_DMA_SAMPLES) {
		return;
	}

	is_recording = true;
	start_freq_mhz = start_freq_mhz_;
	stop_freq_mhz = stop_freq_mhz_;
	freq_step_size_mhz = (stop_freq_mhz - start_freq_mhz_) / num_steps_;
	num_samples_per_step = num_samples_per_step_;
	current_step_num = 0;
	gpr_record_start(start_freq_mhz, num_samples_per_step);
}

/**
 * @brief Gets pointer to data from the GPR if recording isn't in progress
 * @return Pointer to data if recording not in progress, NULL if recording is in progress
 */
static uint32_t* gpr_get_data() {
	uint32_t* data = signal_receiver_get_data(&sig_rec, NULL); // We know how many samples there are
	// Stop reference clock to signal receiver if signal receiver is done recording
	if (data) {
		signal_generator_stop(&sig_rec_reference);
	}
	return data;
}

bool gpr_manager_loop_recording() {
	if (!is_recording) {
		return false;
	}

	uint32_t* data = gpr_get_data();
	// Check if current frequency recording is still in progress
	if (!data) {
		return true;
	}

	// If reached, current recording has completed. Copy into data array
	memcpy(&last_data[current_step_num], data, num_samples_per_step);
	current_step_num++;
	double next_freq_mhz = start_freq_mhz + freq_step_size_mhz * current_step_num;
	// Stop whole recording if the last frequency has completed
	if (next_freq_mhz > stop_freq_mhz) {
		is_recording = false;
		return false;
	}
	// If reached, current frequency recording has stopped but more frequencies must be swept
	gpr_record_start(start_freq_mhz + freq_step_size_mhz * current_step_num, num_samples_per_step);
	return true;
}

bool gpr_manager_get_data(uint32_t** data, double** freqs_mhz, int* actual_num_steps, int* array_samples_per_step, int* actual_samples_per_step) {
	*data = &(last_data[0][0]);
	*freqs_mhz = last_frequencies;
	*actual_num_steps = current_step_num; // Due to behavior of gpr_manager_loop_recording(), current_step_num will end 1 over the last index number
	*array_samples_per_step = SIG_RECEIVER_MAX_DMA_SAMPLES;
	*actual_samples_per_step = num_samples_per_step;

	return !is_recording;
}
