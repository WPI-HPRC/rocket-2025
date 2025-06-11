#include "States.h"

void PreLaunch::initialize_impl() {}

State *PreLaunch::loop_impl() {
    const auto accelData = ctx->mag.getData();
    const auto baroData = ctx->baro.getData();
    
    if (accelData.getLastUpdated() != lastAccelReadingTime) {
        lastAccelReadingTime = accelData.getLastUpdated();
        if (accelDebouncer.update(accelData->accelZ > LAUNCH_THRESHHOLD,
                                        ::millis())) {
            return new Boost(ctx);
        }
    }

    if (!altAverager.isBufferSaturated() && baroData.getLastUpdated() != lastBaroReadingTime) {
        lastBaroReadingTime = baroData.getLastUpdated();
        altAverager.update(baroData->altitude);

        if (altAverager.isBufferSaturated()) {
            ctx->initialAltitude = altAverager.getAvg();
            ctx->errorLogFile.printf("[%u] Initial altitude: %f m\n", ::millis(), ctx->initialAltitude);
        }
    }

    if (currentTime >= 1000) return new CoastWait(ctx);
    return nullptr;
}
