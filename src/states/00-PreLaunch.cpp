#include "States.h"

void PreLaunch::initialize_impl() {}

State *PreLaunch::loop_impl() {
    const auto accelData = ctx->mag.getData();
    if (accelData.getLastUpdated() != lastAccelReadingTime) {
        lastAccelReadingTime = accelData.getLastUpdated();
        if (accelDebouncer.update(accelData->accelZ > LAUNCH_THRESHHOLD,
                                        ::millis())) {
            return new Boost(ctx);
        }
    }
    return nullptr;
}
