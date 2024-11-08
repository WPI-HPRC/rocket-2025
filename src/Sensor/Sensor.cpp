//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"
#include <Arduino.h>


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

