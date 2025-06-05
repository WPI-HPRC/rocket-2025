#include "States.h"

void MainDescent::initialize_impl() {
  prevAltitude = this->ctx->baro.getData()->altitude;
}

State *MainDescent::loop_impl() {
  if (lastBaroReadingTime < ctx->baro.getLastTimePolled()) {
    lastBaroReadingTime = ctx->baro.getLastTimePolled();

    if (firstVelCalculated) {
      avgBaroVel = alpha * (ctx->baro.getData()->altitude - prevAltitude) * this->deltaTime / 1000. + (1 - alpha) * avgBaroVel;
      if (fabs(avgBaroVel) < LANDED_VEL_THRESHHOLD) {
        return new Recovery(this->ctx);
      }
    } else {
      avgBaroVel = (ctx->baro.getData()->altitude - prevAltitude) * this->deltaTime / 1000.;
      firstVelCalculated = true;
    }
  }

  if (this->currentTime >= MAIN_DESCENT_MAX_TIME) {
    return new Recovery(this->ctx);
  }
  
  return nullptr;
}
