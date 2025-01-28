#include "State.h"

#include <Arduino.h>

State::State(Context ctx) : ctx(ctx) {}

void State::initialize() {
    this->startTime = millis();
    initialize_impl();
}

State *State::loop() {
	uint64_t now = millis();
  // These values may be used in the state code
	this->currentTime = now - this->startTime;
	this->deltaTime = now - this->lastLoopTime;
	this->loopCount++;

	State *next = loop_impl();

#ifndef NO_TRANSITION
    return next;
#else
		return nullptr;
#endif
}
