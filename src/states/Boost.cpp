#include "States.h"

void Boost::initialize_impl() {
  Serial.println("Boost initialized!");
}

State_t *Boost::loop_impl() {
  Serial.println("Boost looped");

  return nullptr;
}
