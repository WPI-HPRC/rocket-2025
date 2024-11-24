#include "Rocket/Rocket.h"
#include <Arduino.h>
#include <cstring>

Rocket::Rocket(Time *time) :
time(time),
accelerometer(100, 0),
magnetometer(200, 1) {}

void Rocket::init() {
    sensorManager.addSensor(&accelerometer);
    sensorManager.addSensor(&magnetometer);
    if(!sensorManager.sensorInit()) {
        Serial.println("sensorManager Failed");
    }
    if(!taskScheduler.add(&sensorManager)) {
        Serial.println("task scheduler failed to add SensorManager");
    }

    /*
    if(sensorManager.sensorInit()) {
        Serial.println("all is good, moving on");
    } else {
        Serial.println("oh no... :(");
    }
    if(taskScheduler.add(&sensorManager)) {
        Serial.println("sensor manager added");
    } else {
        Serial.println("sensor manager was not added");
    }
     */
}

void Rocket::iterate() {

    taskScheduler.run();
/*
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
    */
}
