#include "States.h"

void MainDescent::initialize_impl() {
  prevAltitude = this->ctx->baro.getData()->altitude;
}

State *MainDescent::loop_impl() {
  const auto baroData = ctx->baro.getData();

  if (lastBaroReadingTime < baroData.getLastUpdated()) {
    lastBaroReadingTime = baroData.getLastUpdated();

    ewma.update((baroData->altitude - prevAltitude) * (::millis() - lastBaroReadingTime) / 1000.);
    if (velDebouncer.update(std::abs(ewma.getAvg()) < LANDED_VEL_THRESHHOLD, ::millis())) {
      return new Recovery(this->ctx);
    }
  }

  if (this->currentTime >= MAIN_DESCENT_MAX_TIME) {
    ctx->errorLogFile.printf("[%d] MainDescent state timed out\n", ::millis());
    return new Recovery(this->ctx);
  }
  
  return nullptr;
}
