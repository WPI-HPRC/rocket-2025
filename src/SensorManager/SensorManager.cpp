//
// Created by Daniel Coburn on 11/15/24.
//

#include "SensorManager.h"
#include <Arduino.h>

bool SensorManager::addSensor(Sensor *sensorPtr) {
    Serial.println("adding sensor");
        auto it = std::find(sensors.begin(), sensors.end(), sensorPtr);
        // ask about this might be a better way

        if (it == sensors.end()) {
            Serial.println("added...");
            sensors.push_back(sensorPtr);
            return true;
        }
        Serial.print("sensor already exists");
    return false;
}

void** SensorManager::readSensors() {
    Serial.print("read sensors from list of len "); Serial.println(sensors.size());
    // TODO this might be a memory leak issue down the line

    void** data = new void*[sensors.size()];

    for (size_t i = 0; i < sensors.size(); i++) {
        if (sensors[i]->getInitStatus()) {
            Serial.print("I am reading sensor ");
            Serial.println(i);
            long currentTime = timer->millis();
            if (sensors[i]->getLastTimeRead() + sensors[i]->getPollingPeriod() >= currentTime) {
                data[i] = sensors[i]->update(currentTime);

                Serial.print("sensor update from sensor ");
                Serial.println(i);
            }
        } else {
            // could add some error block for diagnostics on telemetry?
            Serial.println("sensor does not exist");
        }
    }
    return data; // data should be freed by the caller
}

void SensorManager::run() {
    //Serial.println("reading sensors...");
    // can put data here somewhere for later use or something
    void** data = readSensors();
    for(size_t i = 0; i < sensors.size(); i++) {
        Serial.println("doing things to data...");
    }
    delete[] data; // TODO move this somewhere else so data can be accessed
}

bool SensorManager::sensorInit() {
    bool success = true;
    for (size_t i = 0; i < sensors.size(); ++i) {
        success = success && sensors[i]->init();
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" init status: ");
        Serial.println(sensors[i]->getInitStatus() ? "SUCCESS" : "FAILURE");
    }
    return success;
}
