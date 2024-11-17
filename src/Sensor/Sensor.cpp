//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

/*
void Sensor::init() {
    initStatus = true;
}
 */


void* Sensor::update() {
    // TODO: Time is going to be handled by the SensorManager
    //long now = time->millis();
    if (initStatus) {
        //lastTimeRead = now;
        //id.timestamp = lastTimeRead;
        return poll();
    }
    return nullptr;
}

/*
bool Sensor::readyToRead() {
    long now = time->millis();
    return (now - lastTimeRead >= pollingPeriod);
}
 */

long Sensor::getPollingPeriod() {
    return pollingPeriod;
}

long Sensor::getLastTimeRead() {
    return lastTimeRead;
}

bool Sensor::getInitStatus() {
    return initStatus;
}
