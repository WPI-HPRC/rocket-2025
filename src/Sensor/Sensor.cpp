//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

/**
 *
 * @param currentTime current time of the rocket
 * @return void*, pointer to the data produced by the sensor
 */
void* Sensor::update(long currentTime) {
    // TODO: Time is going to be handled by the SensorManager
    if (initStatus) {
        lastTimeRead = currentTime;
        return poll();
    }
    return nullptr;
}

/**
 * gets polling period of the sensor in millis
 * @return long, the polling period of the sensor in millis
 */
long Sensor::getPollingPeriod() {
    return pollingPeriod;
}

/**
 * gets the last time the sensor was read in millis
 * @return long, the last time the sensor was read in millis
 */
long Sensor::getLastTimeRead() {
    return lastTimeRead;
}

/**
 * gets if the sensor was initialized
 * @return bool, if the sensor was initialized
 */
bool Sensor::getInitStatus() {
    return initStatus;
}
