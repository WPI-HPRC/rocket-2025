#include "States.h"

void MainDescent::initialize_impl() {
  // prevAltitude = pressureToAltitude(this->ctx->baro->getData()->pressure);
}

State *MainDescent::loop_impl() {
  // float altitude = pressureToAltitude(this->ctx->baro->getData()->pressure);
  // float baroVel = (altitude - prevAltitude) * this->deltaTime / 1000.;
  // if (abs(baroVel) < LANDED_DETECTION_VEL_THRESHHOLD {
  //   return (State *)Recovery(this->ctx);
  // }
  
  return nullptr;
}
