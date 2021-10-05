/*
 * state_drive.h
 *
 * Robot movement is controlled by the operator. No GPR recording occurs
 */

#ifndef STATES_INC_STATE_DRIVE_H_
#define STATES_INC_STATE_DRIVE_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "state_interface.h"

class DriveState : public State {

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

#endif /* STATES_INC_STATE_DRIVE_H_ */
