#include "Rocket/Rocket.h"
#include <Arduino.h>


#include <cstring>

Rocket::Rocket(Time *time) : time(time) {
    sensors = Sensors {
        .s1 = new ExampleSensor(time, 5000),
        .s2 = new ExampleSensor(time, 1000),
    };
}

void Rocket::iterate() {

    Sensor** sensorArray = (Sensor**) &sensors;

    for (size_t i = 0; i < sizeof(Sensors) / sizeof(Sensor*); i++) {
        void* data = sensorArray[i]->update();

        size_t dataSize = sensorArray[i]->sensorDataBytes();
        if (data) {
            char* buffer = new char[dataSize];
            memcpy(buffer, data, dataSize);

            Serial.printf("Data %d: %s @ %d\n", i, buffer, sensorArray[i]->getLastTimeRead());

            delete[] buffer;
        }
    }
}
