//
// Created by Daniel Coburn on 9/27/24.
//

#include "ExampleSensor.h"
#include "Arduino.h"

/**
 * read data from sensor
 * @return void* the data from the sensor
 */
void* ExampleSensor::poll() {
    data.a = rand();
    data.b = rand();

    Serial.println("ExampleSensor poll");

    return &data;
}

/**
 * get size of data
 * @return the size of the data in bytes
 */
size_t ExampleSensor::sensorDataBytes() const {
    return sizeof(ExampleData);
}
