#include "Rocket/Rocket.h"
#include <Arduino.h>

#include <cstring>

Rocket::Rocket(Time *time) : time(time) {

    sensors = Sensors {
      .accelerometer = new Accelerometer(time, 1000 / 100),
    };
}

void Rocket::init() {
    Sensor** sensorArray = (Sensor **) &sensors;
    for (size_t i = 0; i < sizeof(Sensors) / sizeof(Sensor *); i++) {
        sensorArray[i]->init(time, 1000 / 100);
        // do we even need this here???
    }
}

void Rocket::iterate() {

    Sensor** sensorArray = (Sensor**) &sensors;

    for (size_t i = 0; i < sizeof(Sensors) / sizeof(Sensor*); i++) {
        void* data = sensorArray[i]->update();
        //Serial.println(data != nullptr);

        size_t dataSize = sensorArray[i]->sensorDataBytes();
        if (data) {
            sensors.accelerometer->debugData();
            char* buffer = new char[dataSize];
            memcpy(buffer, data, dataSize);
            delete[] buffer;
        }
    }
}
