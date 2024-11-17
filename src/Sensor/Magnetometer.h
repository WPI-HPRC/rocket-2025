//
// Created by Daniel Coburn on 11/14/24.
//

#pragma once

#include "Sensor.h"
#include <optional>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

struct MagnetometerData {
    ID id;
    double x;
    double y;
    double z;
};

class Magnetometer: public Sensor  {
public:
    Magnetometer(long pollingPeriod, int sensorNumber) :
    Sensor(pollingPeriod, sensorNumber) { }

    bool init() override;
    std::optional<struct MagnetometerData> getData();
    void debugData() override;

private:
    MagnetometerData data{};
    SFE_MMC5983MA mag;

protected:
    void* poll() override;
    size_t sensorDataBytes() const override;
};
