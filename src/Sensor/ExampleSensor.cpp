//
// Created by Daniel Coburn on 9/27/24.
//

#include "ExampleSensor.h"
#include "Arduino.h"

void* ExampleSensor::poll() {
    data.a = rand();
    data.b = rand();

    Serial.println("ExampleSensor poll");

    return &data;
}

size_t ExampleSensor::sensorDataBytes() const {
    return sizeof(ExampleData);
}
