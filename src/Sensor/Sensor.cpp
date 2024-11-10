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
    if (now - lastTimeRead >= pollingPeriod) {
        lastTimeRead = now;
        id.timestamp = lastTimeRead;
        return poll();
    }
    return nullptr;
}

long Sensor::getLastTimeRead() {
    return lastTimeRead;
}
