#include "States.h"
#include "boilerplate/Utilities/QuaternionUtils.h"
#include "config.h"

void CoastAirbrake::initialize_impl() {
    velocityFilter.ignoreAbsolute();
}

State *CoastAirbrake::loop_impl() {
    const auto baroData = ctx->baro.getData();
    const auto accData = ctx->accel.getData();
    const auto magData = ctx->mag.getData();
    const auto rotM = QuaternionUtils::quatToRot(ctx->attEkfLogger.getState());
    
    const auto rotatedAccel = rotM * BLA::Matrix<3, 1>{accData->accelX, accData->accelY, accData->accelZ};
    float acc_z_best = rotatedAccel(1);
    float alt_best = baroData->altitude;
    // populate in case no new data to use

    if (accData.getLastUpdated() != lastAccelReadingTime) {
        const auto rotatedAccel = rotM * BLA::Matrix<3, 1>{accData->accelX, accData->accelY, accData->accelZ};
        velocityFilter.updateDelta(rotatedAccel(1) * (::millis() - accData.getLastUpdated()));

        acc_z_best = accData->accelZ;
        lastAccelReadingTime = accData.getLastUpdated();
    }

    // more accurate suposedly so better to use
    if (magData.getLastUpdated() != lastMagReadingTime) {
        const auto rotatedAccel = rotM * BLA::Matrix<3, 1>{accData->accelX, accData->accelY, accData->accelZ};
        velocityFilter.updateDelta(rotatedAccel(1) * (::millis() - magData.getLastUpdated()));

        acc_z_best = magData->accelZ;
        lastMagReadingTime = magData.getLastUpdated();
    }

    if (baroData.getLastUpdated() != lastBaroReadingTime) {
        alt_best = baroData->altitude - ctx->initialAltitude;
        lastBaroReadingTime = baroData.getLastUpdated();
    }

    // compute brake
    float deploy = ctx->airbrakes.deployAmmount(acc_z_best, velocityFilter.getVal(), alt_best);
    
    // deploy brake
    ctx->airbrakes.write(SERVO_MIN + (deploy * (SERVO_MAX - SERVO_MIN)));

    if (currentTime >= COAST_AIRBRAKE_TIME) {
        return new CoastEnd(ctx);
    }
    return nullptr;
}
