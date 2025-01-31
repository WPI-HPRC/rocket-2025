#include "States.h"

void PreLaunch::initialize_impl() {
  Serial.println("PreLaunch initialized!");
}

State_t *PreLaunch::loop_impl() {
  Serial.println("PreLaunch looped");

  if (this->currentTime > 5000) {
    return (State_t *)new Boost(this->ctx);
  }
  return nullptr;
}
