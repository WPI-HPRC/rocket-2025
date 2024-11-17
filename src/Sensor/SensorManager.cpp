//
// Created by Daniel Coburn on 11/15/24.
//

#include "SensorManager.h"
#include <Arduino.h>

bool SensorManager::addSensor(Sensor *sensorPtr) {
    auto it = std::find(sensors.begin(), sensors.end(), sensorPtr);
    // ask about this might be a better way

    if (it == sensors.end()) {
        sensors.push_back(sensorPtr);
        return true;
    }
    Serial.print("sensor already exists");
    return false;
}

void** SensorManager::readSensors() {
    // could sort by next polling time
    if (sensors.empty()) return nullptr;

    void** data = new void*[sensors.size()];

    for (size_t i = 0; i < sensors.size(); ++i) {
        //data[i] = sensors[i]->update();
        // THIS MIGHT CHANGE
        // TODO: update next call time for sensor
        //       - could use something like a hashmap without hash, ask abt
        Serial.print("sensor update from sensor "); Serial.print(i);
    }
    return data; // data should be freed by the caller
}

void SensorManager::run() {
    // can put data here somewhere for later use or something
    readSensors();
}
