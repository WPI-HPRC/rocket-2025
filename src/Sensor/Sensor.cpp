//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

bool init(Time *&, long) {
    return 0;
} // is this gonna make it work? idk

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

