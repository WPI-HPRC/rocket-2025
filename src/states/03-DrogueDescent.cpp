#include "States.h"

void DrogueDescent::initialize_impl() {
  prevAltitude = this->ctx->baro.getData()->altitude;
}

State *DrogueDescent::loop_impl() {
  const auto baroData = ctx->baro.getData();

  if (lastBaroReadingTime < baroData.getLastUpdated()) {
    lastBaroReadingTime = baroData.getLastUpdated();

    if (firstVelCalculated) {
      avgBaroVel = alpha * (baroData->altitude - prevAltitude) * this->deltaTime / 1000. + (1 - alpha) * avgBaroVel;
      if (velDebouncer.update(std::abs(avgBaroVel) < DROGUE_DESCENT_VEL_THRESHHOLD, ::millis())) {
        return new MainDescent(this->ctx);
      }
    } else {
      avgBaroVel = (baroData->altitude - prevAltitude) * this->deltaTime / 1000.;
      firstVelCalculated = true;
    }
  }

  if (this->currentTime >= DROGUE_DESCENT_MAX_TIME) {
    ctx->errorLogFile.printf("[%d] DrougeDescent state timed out\n", ::millis());
    return new MainDescent(this->ctx);
  }
  
  return nullptr;
}
