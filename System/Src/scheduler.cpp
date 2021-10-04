/*
 * scheduler.c
 */

#include "scheduler.h"

#include "state_interface.hpp"
#include "stm32f7xx_hal.h"

#define LOOP_PERIOD_MS 10  // How often a loop of the code should be run

static State* p_current_state;
static State* p_next_state;

typedef enum state_id {
	NUM_STATES = 0,
	UNKNOWN
} state_id;

/**
 * @brief Get the next state based on the current state and its end status
 * @param[in] end_status: What the current state returned as its status
 */
static state_id get_next_state(end_status_t end_status) {
	// Return the current state if there's no change
	if (end_status == end_status_t::NoChange) {
		return (state_id) p_current_state->get_id();
	}
	return state_id::UNKNOWN;
}

void scheduler_run() {

	State* states[] = {
		nullptr
	};

	// Initialize the current and next states
	p_current_state = nullptr;
	p_next_state = nullptr;

	// Helper functions throughout infinite loop
	uint32_t last_time = HAL_GetTick();
	end_status_t end_status = end_status_t::NoChange;

	// Keep running scheduler forever
	while(1) {

		// Limit rate scheduler runs at
		if (p_current_state)
			while((HAL_GetTick() - last_time) < LOOP_PERIOD_MS);
		last_time = HAL_GetTick();

		// Cleanup current state and initialize next state if changing states
		if (p_next_state != p_current_state) {
			if (p_current_state) {
				p_current_state->cleanup();
			}
			if (p_next_state) {
				p_next_state->init();
			}
			p_current_state = p_next_state;
		}

		// Run the current state
		if (p_current_state) {
			end_status = p_current_state->run();
		}

		// Find and set the next state
		state_id next_state = get_next_state(end_status);
		for (State* state : states) {
			if (state->get_id() == next_state) {
				p_next_state = state;
				break;
			}
		}
	}
}
