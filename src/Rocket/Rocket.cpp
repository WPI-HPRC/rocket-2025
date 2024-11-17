#include "Rocket/Rocket.h"
#include <Arduino.h>

#include <cstring>

Rocket::Rocket(Time *time) : time(time) {

    sensors = Sensors {
      .accelerometer = new Accelerometer(1000 / 100, 0), // made up numbers
      .magnetometer = new Magnetometer(2000 / 100, 1) // made up numbers
    };
}

void Rocket::init() {
    Sensor** sensorArray = (Sensor **) &sensors;
    for (size_t i = 0; i < sizeof(Sensors) / sizeof(Sensor *); i++) {
        sensorArray[i]->init();
    }
}

void Rocket::iterate() {

    Sensor** sensorArray = (Sensor**) &sensors;

    for (size_t i = 0; i < sizeof(Sensors) / sizeof(Sensor*); i++) {
        void* data = sensorArray[i]->update();

        size_t dataSize = sensorArray[i]->sensorDataBytes();
        sensorArray[i]->debugData();

        if (data) {
            char* buffer = new char[dataSize];
            memcpy(buffer, data, dataSize);
            delete[] buffer;
        }
    }
}
