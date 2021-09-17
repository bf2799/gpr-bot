/*
 * gpr_manager.c
 */

#include "gpr_manager.h"
#include "peripheral_assigner.h"
#include "signal_generator.h"
#include "signal_receiver.h"
#include "stm32f7xx_hal.h"

static signal_generator_t sig_gen;
static signal_receiver_t sig_rec;

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
			SIG_GEN_ON_GPIO_Port,
			SIG_GEN_ON_Pin,
			SystemCoreClock
	);
	signal_receiver_init(&sig_rec, SIGNAL_RECEIVER_ADC);
}

void gpr_record_start(double freq_mhz, uint32_t num_samples, double pulse_time_us) {
	// Set output frequency of signal generator
	signal_generator_set_output_freq(&sig_gen, freq_mhz);
	// Start recording on signal receiver
	signal_receiver_start(&sig_rec, num_samples);

	// Start signal generator and timer for stopping the signal generator
	uint32_t new_period = (uint32_t) ((double) SystemCoreClock * (double) pulse_time_us / 1000000.0);
	__HAL_TIM_SET_AUTORELOAD(GPR_MANAGER_TIMER, new_period);
	signal_generator_start(&sig_gen);
	HAL_TIM_Base_Start_IT(GPR_MANAGER_TIMER);
	HAL_TIM_RegisterCallback(GPR_MANAGER_TIMER, HAL_TIM_PERIOD_ELAPSED_CB_ID, gpr_manager_sig_gen_cplt);
}

uint32_t* gpr_get_data(uint32_t* num_samples) {
	return signal_receiver_get_data(&sig_rec, num_samples);
}
