/*
 * state_record.cpp
 */

#include "state_record.h"

void RecordState::init() {

}

end_status_t RecordState::run() {
	// For now, don't actually record, just immediately return that recording is complete
	return end_status_t::RecordingComplete;
}

void RecordState::cleanup() {

}
