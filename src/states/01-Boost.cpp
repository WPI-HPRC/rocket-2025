#include "States.h"

void Boost::initialize_impl() {}

State *Boost::loop_impl() {
    // NOTE: the lack of absolute value here is intentional: we may experience
    // negative z acceleration due to drag which is larger in magnitude than the
    // threshhold
    if (this->ctx->mag.getData()->accelZ < BURNOUT_THRESHHOLD || this->currentTime >= BOOST_MAX_TIME) {
        return new Coast(this->ctx);
    }
    return nullptr;
}
