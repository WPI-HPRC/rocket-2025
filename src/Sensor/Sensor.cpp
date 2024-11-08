//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

bool Sensor::init(Time*& timePtr, long pollingPeriod) {
    this->time = timePtr;
    this->pollingPeriod = pollingPeriod;
    this->lastTimeRead = timePtr->millis();

    return true;
}

void* Sensor::update() {
    long now = time->millis();
    if (now - lastTimeRead >= pollingPeriod) {
        lastTimeRead = now;
        return poll();
    }
    return nullptr;
}

long Sensor::getLastTimeRead() {
    return lastTimeRead;
}
