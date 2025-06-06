#include "States.h"

void MainDescent::initialize_impl() {
  prevAltitude = this->ctx->baro.getData()->altitude;
}

State *MainDescent::loop_impl() {
  const auto baroData = ctx->baro.getData();

  if (lastBaroReadingTime < baroData.getLastUpdated()) {
    lastBaroReadingTime = baroData.getLastUpdated();

    if (firstVelCalculated) {
      avgBaroVel = alpha * (baroData->altitude - prevAltitude) * this->deltaTime / 1000. + (1 - alpha) * avgBaroVel;
      if (fabs(avgBaroVel) < LANDED_VEL_THRESHHOLD) {
        return new Recovery(this->ctx);
      }
    } else {
      avgBaroVel = (baroData->altitude - prevAltitude) * this->deltaTime / 1000.;
      firstVelCalculated = true;
    }
  }

  if (this->currentTime >= MAIN_DESCENT_MAX_TIME) {
    ctx->errorLogFile.printf("[%d] MainDescent state timed out\n", ::millis());
    return new Recovery(this->ctx);
  }
  
  return nullptr;
}
