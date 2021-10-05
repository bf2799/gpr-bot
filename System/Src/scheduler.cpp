/*
 * scheduler.c
 */

#include "scheduler.h"

#include "state_disabled.h"
#include "state_drive.h"
#include "state_record.h"
#include "state_relocate.h"
#include "stm32f7xx_hal.h"

#define LOOP_PERIOD_MS 10  // How often a loop of the code should be run

static State* p_current_state;
static State* p_next_state;

typedef enum state_id {
	Disabled,
	Drive,
	Record,
	Relocate,
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
	switch(p_current_state->get_id()) {
	case state_id::Disabled:
		switch(end_status) {
		default:
			break;
		}
		break;
	case state_id::Drive:
		switch(end_status) {
		default:
			break;
		}
		break;
	case state_id::Record:
		switch(end_status) {
		default:
			break;
		}
		break;
	case state_id::Relocate:
		switch(end_status) {
		default:
			break;
		}
		break;
	default:
		break;
	}
	return state_id::UNKNOWN;
}

void scheduler_run() {

	DisabledState disabled_state = DisabledState(state_id::Disabled);
	DriveState drive_state = DriveState(state_id::Drive);
	RecordState record_state = RecordState(state_id::Record);
	RelocateState relocate_state = RelocateState(state_id::Relocate);

	State* states[] = {
		&disabled_state,
		&drive_state,
		&record_state,
		&relocate_state
	};

	// Initialize the current and next states
	p_current_state = nullptr;
	p_next_state = &disabled_state;

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
