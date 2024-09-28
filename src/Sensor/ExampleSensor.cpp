//
// Created by Daniel Coburn on 9/27/24.
//

#include "ExampleSensor.h"
#include "Arduino.h"

void* ExampleSensor::poll() {
    data.a = rand();
    data.b = rand();

    return &data;
}