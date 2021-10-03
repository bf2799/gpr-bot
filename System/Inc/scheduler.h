/*
 * scheduler.h
 *
 * Entrypoint of custom software.
 * Defines bare-metal architecture for system with scheduler design pattern.
 * Ensures perception manager runs and passes "perception" to planning manager.
 * Ensures planning manager runs and passes "perception" and "plan" to control manager.
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

/**
 * Runs the scheduler in an infinite loop
 */
void scheduler_run();

#endif /* INC_SCHEDULER_H_ */
