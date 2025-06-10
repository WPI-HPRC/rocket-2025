#include "States.h"
#include "boilerplate/Utilities/QuaternionUtils.h"

void CoastWait::initialize_impl() {
    prevAltitude = ctx->baro.getData()->altitude;
}

State *CoastWait::loop_impl() {
    const auto baroData = ctx->baro.getData();
    const auto magData = ctx->mag.getData();

    // more accurate suposedly so better to use
    if (magData.getLastUpdated() != lastMagReadingTime) {
        const auto rotM = QuaternionUtils::quatToRot(ctx->attEkfLogger.getState());
        const auto rotatedAccel = rotM * BLA::Matrix<3, 1>{magData->accelX, magData->accelY, magData->accelZ};
        // FIXME: find the correct entry of this vector
        velocityFilter.updateDelta(rotatedAccel(2) * (::millis() - magData.getLastUpdated()));

        lastMagReadingTime = magData.getLastUpdated();
    }

    if (baroData.getLastUpdated() != lastBaroReadingTime) {
        ewma.update((baroData->altitude - prevAltitude) *
                    (::millis() - lastBaroReadingTime) / 1000.);
        velocityFilter.updateAbsolute(ewma.getAvg());

        lastBaroReadingTime = baroData.getLastUpdated();
    }

    if (currentTime >= WAIT_AFTER_BURNOUT) {
        return new CoastAirbrake(ctx, velocityFilter);
    }
    return nullptr;
}
