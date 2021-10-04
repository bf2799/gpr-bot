/*
 * state_interface.hpp
 */

#ifndef INC_STATE_INTERFACE_HPP_
#define INC_STATE_INTERFACE_HPP_

#include <cstdint>

typedef enum {
	NoChange
} end_status_t;

class State {

    public:
		State(int id): id_(id) { }
		virtual ~State() = default;

		/**
		 * @brief Returns given ID of this state so higher level can track State pointers instead of IDs
		 * @return Given ID of state
		 */
		int get_id() { return id_; }

		/**
		 * @brief Runs this states
		 * @return End status of the current loop
		 */
		virtual end_status_t run();

		/**
		 * @brief Actions that occur on initialization of state
		 */
        virtual void init(void);

        /**
         * @brief Actions that occur at the end of the state
         */
        virtual void cleanup(void);

    private:
        int id_;
};

#endif /* INC_STATE_INTERFACE_HPP_ */
