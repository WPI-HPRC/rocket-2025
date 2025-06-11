#include "States.h"
#include "boilerplate/Utilities/QuaternionUtils.h"

void CoastWait::initialize_impl() {
    prevAltitude = ctx->baro.getData()->altitude;
    lastBaroReadingTime = ctx->baro.getLastTimePolled();
    deltaV_ewma.update(0);
}

State *CoastWait::loop_impl() {
    const auto baroData = ctx->baro.getData();
    const auto magData = ctx->mag.getData();
    static bool firstBaro = true;

    if (currentTime <= 1000) return nullptr;

    // more accurate suposedly so better to use
    if (magData.getLastUpdated() != lastMagReadingTime) {
        const auto rotM = QuaternionUtils::quatToRot(ctx->attEkfLogger.getState());
        const auto accel = BLA::Matrix<3, 1>{magData->accelX, magData->accelY, magData->accelZ};
        const auto rotatedAccel = rotM * accel;
        Serial.println(accel);
        Serial.println(rotatedAccel);
        float acc = rotatedAccel(2) + 1;
        if (std::abs(acc) <= 0.05) acc = 0;
        acc = -g * (acc) * (magData.getLastUpdated() - lastMagReadingTime) / 1000.;
        Serial.println("Accel delta v: " + String(acc, 3));
        deltaV_ewma.update(acc);
        Serial.println("Accel delta v filtered: " + String(deltaV_ewma.getAvg(), 3));

        velocityFilter.updateDelta(deltaV_ewma.getAvg());
        Serial.print("Filtered vel: "); Serial.println(velocityFilter.getVal(), 5);

        lastMagReadingTime = magData.getLastUpdated();
    }

    if (baroData.getLastUpdated() != lastBaroReadingTime) {
        if (firstBaro) {
            firstBaro = false;
        } else {
            ewma.update((baroData->altitude - prevAltitude) *
                        (baroData.getLastUpdated() - lastBaroReadingTime) / 1000.);
            velocityFilter.updateAbsolute(ewma.getAvg());

            Serial.println("Baro vel: " + String((baroData->altitude - prevAltitude) *
                        (baroData.getLastUpdated() - lastBaroReadingTime) / 1000., 3));

            Serial.print("Filtered vel: "); Serial.println(velocityFilter.getVal(), 5);
        }

        prevAltitude = baroData->altitude;
        lastBaroReadingTime = baroData.getLastUpdated();
    }

    if (currentTime >= WAIT_AFTER_BURNOUT) {
        return new CoastAirbrake(ctx, velocityFilter);
    }
}
