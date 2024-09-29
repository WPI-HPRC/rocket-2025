//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "Sensor.h"

struct ExampleData {
    int a;
    int b;
};

class ExampleSensor: public Sensor {
    using Sensor::Sensor;

private:
    ExampleData data;

protected:
    void* poll() override;

    size_t sensorDataBytes() const override;

};
