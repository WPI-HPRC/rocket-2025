#pragma once

#include "Sensor/Accelerometer.h"
#include "Sensor/Magnetometer.h"
// #include "Sensor/ExampleSensor.h"

struct Sensors {
    Accelerometer* accelerometer;
    Magnetometer* magnetometer;
};
