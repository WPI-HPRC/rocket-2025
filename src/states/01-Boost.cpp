#include "States.h"

void Boost::initialize_impl() {
}

State *Boost::loop_impl() {
  // if (abs(this->ctx->acc->getData()->accelZ) < BURNOUT_THRESHHOLD) {
  //   return (State *)new Coast(this->ctx);
  // }
  return nullptr;
}
