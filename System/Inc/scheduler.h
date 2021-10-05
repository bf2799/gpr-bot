/*
 * scheduler.h
 *
 * Entrypoint of custom software
 * Controls initialization, running, and cleanup of current state
 * Ensures code loops run at a fixed rate for best robot perception and control
 * Contains primary state machine, finding next state based on current state and its "end status"
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#ifdef __cplusplus
extern "C"{
#endif

/**
 * Runs the scheduler in an infinite loop
 */
void scheduler_run();

#ifdef __cplusplus
}
#endif

#endif /* INC_SCHEDULER_H_ */
