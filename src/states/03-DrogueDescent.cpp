#include "States.h"

void DrogueDescent::initialize_impl() {
  prevAltitude = this->ctx->baro.getData()->altitude;
}

State *DrogueDescent::loop_impl() {
  if (lastBaroReadingTime < ctx->baro.getLastTimePolled()) {
    lastBaroReadingTime = ctx->baro.getLastTimePolled();

    if (firstVelCalculated) {
      avgBaroVel = alpha * (ctx->baro.getData()->altitude - prevAltitude) * this->deltaTime / 1000. + (1 - alpha) * avgBaroVel;
      if (fabs(avgBaroVel) < (fabs(MAIN_DESCENT_VELOCITY) + fabs(DROGUE_DESCENT_VELOCITY)) / 2) {
        return new MainDescent(this->ctx);
      }
    } else {
      avgBaroVel = (ctx->baro.getData()->altitude - prevAltitude) * this->deltaTime / 1000.;
      firstVelCalculated = true;
    }
  }

  if (this->currentTime >= DROGUE_DESCENT_MAX_TIME) {
    return new MainDescent(this->ctx);
  }
  
  return nullptr;
}
