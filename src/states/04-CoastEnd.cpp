#include "FlightParams.h"
#include "States.h"

void CoastEnd::initialize_impl() {
  prevAltitude = this->ctx->baro.getData()->altitude;
}

State *CoastEnd::loop_impl() {
  const auto baroData = ctx->baro.getData();

  if (lastBaroReadingTime != baroData.getLastUpdated()) {
    lastBaroReadingTime = baroData.getLastUpdated();

    ewma.update((baroData->altitude - prevAltitude) * (::millis() - lastBaroReadingTime) / 1000.);
    if (velDebouncer.update(std::abs(ewma.getAvg()) < APOGEE_VEL_THRESHHOLD, ::millis())) {
      return new DrogueDescent(this->ctx);
    }
  }

  if (this->currentTime >= COAST_MAX_TIME - WAIT_AFTER_BURNOUT - COAST_AIRBRAKE_TIME) {
    ctx->errorLogFile.printf("[%d] Coast state timed out\n", ::millis());
    return new DrogueDescent(this->ctx);
  }
  return nullptr;
}
