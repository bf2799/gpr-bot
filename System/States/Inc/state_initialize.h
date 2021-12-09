/*
 * state_initialize.h
 *
 * Robot initializes all hardware and managers
 */

#ifndef STATES_INC_STATE_INITIALIZE_H_
#define STATES_INC_STATE_INITIALIZE_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "state_interface.h"

class InitializeState : public State {

	public:
		using State::State;
		using State::get_id;

		void init(void) override;

		end_status_t run(void) override;

		void cleanup(void) override;

	private:

		static constexpr double SEARCH_AREA_WIDTH_M = 10;
		static constexpr double SEARCH_AREA_LENGTH_M = 10;
		static constexpr double NUM_PASSES = 10;
		static constexpr double STOPS_PER_PASS = 0;
};

#ifdef __cplusplus
}
#endif

#endif /* STATES_INC_STATE_INITIALIZE_H_ */
