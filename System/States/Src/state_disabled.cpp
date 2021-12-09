/*
 * state_disabled.cpp
 */

#include "state_disabled.h"
#include "stm32f7xx_hal.h"

void DisabledState::init() {

}

end_status_t DisabledState::run() {
	return end_status_t::Enabled;
}

void DisabledState::cleanup() {

}
