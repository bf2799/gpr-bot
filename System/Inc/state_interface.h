/*
 * state_interface.h
 */

#ifndef INC_STATE_INTERFACE_H_
#define INC_STATE_INTERFACE_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <cstdint>

typedef enum {
	NoChange,
	InitializationComplete,
	RecordingComplete,
	SystemDisabled,
	SystemEnabled,
	TrajectoryComplete
} end_status_t;

class State {

    public:
		State(int id): id_(id) { }
		~State() = default;

		/**
		 * @brief Returns given ID of this state so higher level can track State pointers instead of IDs
		 * @return Given ID of state
		 */
		int get_id() { return id_; }

		/**
		 * @brief Runs this states
		 * @return End status of the current loop
		 */
		virtual end_status_t run() = 0;

		/**
		 * @brief Actions that occur on initialization of state
		 */
        virtual void init(void) = 0;

        /**
         * @brief Actions that occur at the end of the state
         */
        virtual void cleanup(void) = 0;

    private:
        int id_;
};

#ifdef __cplusplus
}
#endif

#endif /* INC_STATE_INTERFACE_H_ */
