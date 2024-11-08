//
// Created by Daniel Coburn on 10/31/24.
//

#pragma once
#include <ICM42688.h> // need to add ??
#include "Sensor.h"

const int addr = 0x68;

struct Data {
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};

class Accelerometer: public Sensor {

public:
    Accelerometer(Time* timePtr, long pollingPeriod) :
        Sensor(timePtr, pollingPeriod),
        icm42688(Wire, addr) { }

    bool init(Time *&, long);

private:
    ICM42688 icm42688;
    Data data;

protected:
    void* poll() override;
    size_t sensorDataBytes() const;
};
