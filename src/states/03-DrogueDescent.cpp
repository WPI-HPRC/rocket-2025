#include "States.h"

void DrogueDescent::initialize_impl() {
  // prevAltitude = pressureToAltitude(this->ctx->baro->getData()->pressure);
}

State *DrogueDescent::loop_impl() {
  // float altitude = pressureToAltitude(this->ctx->baro->getData()->pressure);
  // float baroVel = (altitude - prevAltitude) * this->deltaTime / 1000.;
  // if (abs(baroVel) < (MAIN_DESCENT_VELOCITY + DROGUE_DESCENT_VELOCITY) / 2.0) {
  //   return (State *)MainDescent(this->ctx);
  // }
  
  return nullptr;
}
