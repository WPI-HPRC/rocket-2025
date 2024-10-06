//
// Created by Daniel Coburn on 10/5/24.
//

#include "Accelerometer.h"
#include <Wire.h>
using namespace Accelerometer;
const addr = 0x68;

bool Accelerometer::init(Context *ctx) {

    ctx->icm42688(Wire, addr)

    if(ctx->icm42688.begin() != 1)
        return false;

    ctx->icm42688.setAccelFS(ICM42688::gpm16);
    ctx->icm42688.setGyroFS(ICM42688:dps250);

    ctx->icm42688.setAccelODR(ICM42688:odr100);
    ctx->icm42688.setGyroORD(ICM42688::ord100);

    return true;
}

Data Accelerometer::poll(Context *ctx) {
    ctx->icm42688.getAGT();

    return Data{
            .accX = ctx->icm42688.accX(),
            .accY = ctx->icm42688.accY(),
            .accZ = ctx->icm42688.accZ(),
            .gyroX = ctx->icm42688.gyrX(),
            .gyroY = ctx->icm42688.gyrY(),
            .gyroZ = ctx->icm42688.gyrZ(),
    };
}