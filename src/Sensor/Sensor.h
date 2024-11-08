//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "services/Time.h"

#include <stdlib.h>

class Sensor {

protected:
    virtual void* poll() = 0;
    long lastTimeRead;
    long pollingPeriod;
    Time* time;

public:

    Sensor(Time*& timePtr, long pollingPeriod) :
        time(timePtr),
        pollingPeriod(pollingPeriod),
        lastTimeRead(time->millis())
    {}

    bool init(Time*&, long);

    void* update();
    long getLastTimeRead();

    virtual size_t sensorDataBytes() const = 0;
    virtual ~Sensor() = default; /* what is this for */

    void* data; // what is this
};