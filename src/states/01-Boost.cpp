#include "States.h"

void Boost::initialize_impl() {}

State *Boost::loop_impl() {
    const auto accelData = ctx->mag.getData();
    if (lastAccelReadingTime != accelData.getLastUpdated()) {
        lastAccelReadingTime = accelData.getLastUpdated();
        // NOTE: the lack of absolute value here is intentional: we may experience
        // negative z acceleration due to drag which is larger in magnitude than the
        // threshhold
        if (boostAccelDebouncer.update(accelData->accelZ < BURNOUT_THRESHHOLD, ::millis())) {
            return new Coast(this->ctx);
        }
        if (this->currentTime >= BOOST_MAX_TIME) {
            ctx->errorLogFile.printf("[%d] Boost state timed out\n", ::millis());
            return new Coast(this->ctx);
        }
    }
    // FIXME: possible Abort transition here (or flag to say no airbrakes)
    return nullptr;
}
