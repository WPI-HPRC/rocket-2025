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
private:
    void* data;

protected:
    virtual void* poll() = 0;
    long lastTimeRead;
    long pollingPeriod;
    Time* time;
    ID id;

public:

    Sensor(Time* timePtr, long pollingPeriod) :
        time(timePtr),
        pollingPeriod(pollingPeriod),
        lastTimeRead(0)
    {}

    virtual bool init(Time*&, long); // clean
    void* update();
    long getLastTimeRead();

    virtual size_t sensorDataBytes() const = 0;
    virtual ~Sensor() = default;
};