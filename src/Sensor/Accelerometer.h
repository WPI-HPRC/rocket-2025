//
// Created by Daniel Coburn on 10/5/24.
//
#pragma once
#include <ICM42688.h> // need to add
#include <Sensor.h>

namespace Accelerometer {

    struct Context {
        ICM42688 icm42688;
    };

    struct Data {
        float accX;
        float accY;
        float accZ;
        float gyroX;
        float gyroY;
        float gyroZ;
    };

    bool init(Context *ctx);
    Data poll(Context *ctx);

}
