//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "services/Time.h"
#include <stdlib.h>

struct ID {
    long timestamp;
    // would having a feild for telemetry use here be usefull??
};

class Sensor {

protected:
    long lastTimeRead;
    long pollingPeriod;
    Time* time;
    virtual void* poll() = 0;

private:
    ID id{};

public:
    Sensor(Time* timePtr, long pollingPeriod) :
        time(timePtr),
        pollingPeriod(pollingPeriod),
        lastTimeRead(0)
    {}

    bool readyToRead();

    virtual bool init() = 0;
    virtual void debugData() = 0;
    void* update();
    long getLastTimeRead();
    virtual size_t sensorDataBytes() const = 0;
    virtual ~Sensor() = default;
};