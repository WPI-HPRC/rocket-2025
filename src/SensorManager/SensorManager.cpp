//
// Created by Daniel Coburn on 11/15/24.
//

#include "SensorManager.h"
#include <Arduino.h>

/**
 * public \n
 * adds a sensor pointer to the SensorManger
 * @param sensorPtr pointer to the sensor to add
 * @return true if added, false if not
 */
bool SensorManager::addSensor(Sensor *sensorPtr) {
    Serial.println("adding sensor");
        auto it = std::find(sensors.begin(), sensors.end(), sensorPtr);

        if (it == sensors.end()) {
            Serial.println("added...");
            sensors.push_back(sensorPtr);
            return true;
        }
        Serial.print("sensor already exists");
    return false;
}

/**
 * private \n
 * reads all of the sensors that are ready to be read
 * @return void**, array of pointers to data
 */
void** SensorManager::readSensors() {
    void** data = new void*[sensors.size()];

    for (size_t i = 0; i < sensors.size(); i++) {
        if (sensors[i]->getInitStatus()) {
            long currentTime = timer->millis();
            if (sensors[i]->getLastTimeRead() + sensors[i]->getPollingPeriod() >= currentTime) {
                data[i] = sensors[i]->update(currentTime);

                Serial.print("sensor update from sensor ");
                Serial.println(i);
            }
        } else {
            // could add some error block for diagnostics on telemetry?
        }
    }
    return data; // data should be freed by the caller
}

/**
 * public \n
 * reads all of the sensors
 * data should be freed by you
 */
void SensorManager::run() {
    void** data = readSensors();
    //for(size_t i = 0; i < sensors.size(); i++) {
    //    Serial.println("doing things to data...");
    //}
    delete[] data; // TODO move this somewhere else so data can be accessed
}

/**
 * public \n
 * initializes sensors that the SensorManager has
 * @return true if all sensors where initialized successfully, false if not
 */
bool SensorManager::sensorInit() {
    bool success = true;
    for (size_t i = 0; i < sensors.size(); ++i) {
        sensors[i]->init();
        success = success && sensors[i]->getInitStatus();
        Serial.println(sensors[i]->getInitStatus() ? "SUCCESS" : "FAILURE");
    }
    return success;
}
