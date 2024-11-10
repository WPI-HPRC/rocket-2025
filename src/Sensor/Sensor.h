//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "services/Time.h"
#include <stdlib.h>

struct ID {
    long timestamp;
};

class Sensor {

protected:
    long lastTimeRead;
    long pollingPeriod;
    Time* time;
    ID id{};

    virtual void* poll() = 0;

public:
    Sensor(Time* timePtr, long pollingPeriod) :
        time(timePtr),
        pollingPeriod(pollingPeriod),
        lastTimeRead(0)
    {}

    virtual bool init() = 0;
    void* update();
    long getLastTimeRead();
    virtual size_t sensorDataBytes() const = 0;
    virtual ~Sensor() = default;
};