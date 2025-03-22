#include "States.h"

void Coast::initialize_impl() {
  // prevAltitude = pressureToAltitude(this->ctx->baro->getData()->pressure);
}

State *Coast::loop_impl() {
  // Some sort of active airbrake control here

  // float altitude = pressureToAltitude(this->ctx->baro->getData()->pressure);
  // float baroVel = (altitude - prevAltitude) * this->deltaTime / 1000.;
  // if (abs(baroVel) < APOGEE_THRESHHOLD) {
  //   return (State *)DrogueDescent(this->ctx);
  // }
  return nullptr;
}
