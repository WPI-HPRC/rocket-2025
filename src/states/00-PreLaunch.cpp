#include "States.h"

void PreLaunch::initialize_impl() {}

State *PreLaunch::loop_impl() {
    const auto accelData = ctx->accel.getData();
    if (accelData.getLastUpdated() != lastAccelReadingTime) {
        lastAccelReadingTime = accelData.getLastUpdated();
        if (launchAccelDebouncer.update(accelData->accelZ > LAUNCH_THRESHHOLD_G,
                                        ::millis())) {
            return new Boost(ctx);
        }
    }
    return nullptr;
}
