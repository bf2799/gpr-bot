/*
 * scheduler.c
 */

#include "scheduler.h"

#include "state_disabled.h"
#include "state_drive.h"
#include "state_initialize.h"
#include "state_record.h"
#include "stm32f7xx_hal.h"

#define LOOP_PERIOD_MS 10  // How often a loop of the code should be run

static State* p_current_state;
static State* p_next_state;

typedef enum state_id {
	Disabled = 0,
	Drive,
	Initialize,
	Record,
	NUM_STATES,
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
		case end_status_t::SystemEnabled:
			return state_id::Drive;
		default:
			break;
		}
		break;
	case state_id::Drive:
		switch(end_status) {
		case end_status_t::SystemDisabled:
			return state_id::Disabled;
		case end_status_t::TrajectoryComplete:
			return state_id::Record;
		default:
			break;
		}
		break;
	case state_id::Initialize:
		switch(end_status) {
		case end_status_t::InitializationComplete:
			return state_id::Disabled;
		default:
			break;
		}
		break;
	case state_id::Record:
		switch(end_status) {
		case end_status_t::RecordingComplete:
			return state_id::Drive;
		case end_status_t::SystemDisabled:
			return state_id::Disabled;
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

	InitializeState initialize_state = InitializeState(state_id::Initialize);
	DisabledState disabled_state = DisabledState(state_id::Disabled);
	DriveState drive_state = DriveState(state_id::Drive);
	RecordState record_state = RecordState(state_id::Record);

	State* states[] = {
		&initialize_state,
		&disabled_state,
		&drive_state,
		&record_state,
	};

	// Initialize the current and next states
	p_current_state = nullptr;
	p_next_state = &initialize_state;

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
