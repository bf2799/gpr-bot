/*
 * state_record.h
 *
 * Robot is stationary and recording GPR data
 */

#ifndef STATES_INC_STATE_RECORD_H_
#define STATES_INC_STATE_RECORD_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "state_interface.h"

class RecordState : public State {

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

#endif /* STATES_INC_STATE_RECORD_H_ */
