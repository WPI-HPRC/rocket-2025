#include "States.h"

void Coast::initialize_impl() {
  prevAltitude = this->ctx->baro.getData()->altitude;
}

State *Coast::loop_impl() {
  // FIXME: Some sort of active airbrake control here

  if (lastBaroReadingTime < ctx->baro.getLastTimePolled()) {
    lastBaroReadingTime = ctx->baro.getLastTimePolled();

    if (firstVelCalculated) {
      avgBaroVel = alpha * (ctx->baro.getData()->altitude - prevAltitude) * this->deltaTime / 1000. + (1 - alpha) * avgBaroVel;
      if (fabs(avgBaroVel) < APOGEE_VEL_THRESHHOLD) {
        return new DrogueDescent(this->ctx);
      }
    } else {
      avgBaroVel = (ctx->baro.getData()->altitude - prevAltitude) * this->deltaTime / 1000.;
      firstVelCalculated = true;
    }
  }

  if (this->currentTime >= COAST_MAX_TIME) {
    return new DrogueDescent(this->ctx);
  }
  return nullptr;
}
