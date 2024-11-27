//
// Created by Daniel Coburn on 10/31/24.
//

#pragma once
#include "../Sensor.h"
#include <optional>
#include <ICM42688.h>

const int addr = 0x68;

struct AccelerometerData {
    ID id;
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};

class Accelerometer: public Sensor {

public:
    Accelerometer(long pollingPeriod, int sensorNumber) :
        Sensor(pollingPeriod, sensorNumber),
        icm42688(Wire, addr) { }

    bool init() override;
    std::optional<AccelerometerData> getData();
    void debugData() override;

private:
    AccelerometerData data{};
    ICM42688 icm42688;

protected:
    void* poll() override;
    size_t sensorDataBytes() const override;
};
