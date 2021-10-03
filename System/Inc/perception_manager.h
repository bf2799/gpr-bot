/*
 * perception_manager.h
 *
 * Tracks what information should be gathered from environment (what managers should be run)
 * Conglomerates user, localization, and radar input data into a single perception vector
 */

#ifndef INC_PERCEPTION_MANAGER_H_
#define INC_PERCEPTION_MANAGER_H_


typedef struct perception_t {
	void* na;
} perception_t;

/**
 * @brief Runs a single loop of the perception manager
 * @return Perception generated in this run through
 */
perception_t* perception_manager_run();

#endif /* INC_PERCEPTION_MANAGER_H_ */
