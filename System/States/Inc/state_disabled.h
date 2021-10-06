/*
 * state_disabled.h
 *
 * Robot is not allowed to move or record for safety reasons, but data can be sent out if previously recorded
 */

#ifndef STATES_INC_STATE_DISABLED_H_
#define STATES_INC_STATE_DISABLED_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "state_interface.h"

class DisabledState : public State {

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

#endif /* STATES_INC_STATE_DISABLED_H_ */
