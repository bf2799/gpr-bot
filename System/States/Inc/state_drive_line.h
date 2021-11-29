/*
 * state_drive_line.h
 *
 * Robot drives straight along a line to get to the next recording point
 */

#ifndef STATES_INC_STATE_DRIVE_LINE_H_
#define STATES_INC_STATE_DRIVE_LINE_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "state_interface.h"

class DriveLineState : public State {

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

#endif /* STATES_INC_STATE_DRIVE_LINE_H_ */
