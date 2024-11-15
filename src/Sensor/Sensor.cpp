//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

/*
bool Sensor::init() {
    return true;
}
*/

void* Sensor::update() {
    long now = time->millis();
    if (readyToRead()) {
        lastTimeRead = now;
        id.timestamp = lastTimeRead;
        return poll();
    }
    return nullptr;
}

bool Sensor::readyToRead() {
    long now = time->millis();
    return (now - lastTimeRead >= pollingPeriod);
}

long Sensor::getLastTimeRead() {
    return lastTimeRead;
}
