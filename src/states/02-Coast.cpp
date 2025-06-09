#include "States.h"

void Coast::initialize_impl() {
  prevAltitude = this->ctx->baro.getData()->altitude;
}

State *Coast::loop_impl() {
  // FIXME: Some sort of active airbrake control here

  const auto baroData = ctx->baro.getData();

  if (lastBaroReadingTime != baroData.getLastUpdated()) {
    lastBaroReadingTime = baroData.getLastUpdated();

    if (firstVelCalculated) {
      avgBaroVel = alpha * (baroData->altitude - prevAltitude) * this->deltaTime / 1000. + (1 - alpha) * avgBaroVel;
      if (velDebouncer.update(std::abs(avgBaroVel) < APOGEE_VEL_THRESHHOLD, ::millis())) {
        return new DrogueDescent(this->ctx);
      }
    } else {
      avgBaroVel = (baroData->altitude - prevAltitude) * this->deltaTime / 1000.;
      firstVelCalculated = true;
    }
  }

  if (this->currentTime >= COAST_MAX_TIME) {
    ctx->errorLogFile.printf("[%d] Coast state timed out\n", ::millis());
    return new DrogueDescent(this->ctx);
  }
  return nullptr;
}
