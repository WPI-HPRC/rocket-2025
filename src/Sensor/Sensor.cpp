//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

/*
void Sensor::init() {
    initStatus = true;
}
 */


void* Sensor::update(long currentTime) {
    // TODO: Time is going to be handled by the SensorManager
    //long now = time->millis();
    if (initStatus) {
        lastTimeRead = currentTime;
        return poll();
    }
    return nullptr;
}

long Sensor::getPollingPeriod() {
    return pollingPeriod;
}

long Sensor::getLastTimeRead() {
    return lastTimeRead;
}

bool Sensor::getInitStatus() {
    return initStatus;
}
