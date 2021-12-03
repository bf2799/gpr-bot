/*
 * state_drive.h
 *
 * Robot drives to get to the next recording point
 */

#ifndef STATES_INC_STATE_DRIVE_H_
#define STATES_INC_STATE_DRIVE_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "state_interface.h"

#include <stdbool.h>

class DriveState : public State {

	public:
		using State::State;
		using State::get_id;

		void init(void) override;

		end_status_t run(void) override;

		void cleanup(void) override;

	private:
		bool line_complete_ = false;
		bool search_complete_ = false;
};

#ifdef __cplusplus
}
#endif

#endif /* STATES_INC_STATE_DRIVE_H_ */
