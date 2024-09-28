//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

Sensor::Sensor(Time* time, long pollingPeriod):
    time(time),
    pollingPeriod(pollingPeriod),
    lastTimeRead(time->millis())
{}

void* Sensor::update() {

    long now = time->millis();
    if (now - lastTimeRead >= pollingPeriod) {
        lastTimeRead = now;
        return poll();
    }

    return nullptr;
}