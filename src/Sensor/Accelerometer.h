//
// Created by Daniel Coburn on 10/31/24.
//

#pragma once
#include <ICM42688.h> // need to add ??
#include "Sensor.h"

struct Data {
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};

class Accelerometer: public Sensor {
    using Sensor::Sensor;

private:
    Data data;

protected:
    void* poll() override;

    size_t sensorDataBytes() const override;
};