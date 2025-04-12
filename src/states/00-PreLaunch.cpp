#include "States.h"

constexpr int LAUNCH_THRESHHOLD = 5;

void PreLaunch::initialize_impl() {
}

State *PreLaunch::loop_impl() {  
  // Serial.println("PreLaunch Looped");
  // if (this->ctx->accel->getData().zAcc > LAUNCH_THRESHHOLD) {
  //   return (State *)new Boost(this->ctx);
  // }
  return nullptr;
}
