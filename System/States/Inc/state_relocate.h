/*
 * state_relocate.h
 *
 * Robot is automatically driving to a new location
 */

#ifndef STATES_INC_STATE_RELOCATE_H_
#define STATES_INC_STATE_RELOCATE_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "state_interface.h"

class RelocateState : public State {

	public:
		using State::State;
		using State::get_id;

		void init(void) override;

		end_status_t run(void) override;

		void cleanup(void) override;
};

#ifdef __cplusplus
}
#endif

#endif /* STATES_INC_STATE_RELOCATE_H_ */
