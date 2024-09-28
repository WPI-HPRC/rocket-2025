//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "services/Time.h"

#include <stdlib.h>

class Sensor {

private:
    Time* time;
    long lastTimeRead;
    long pollingPeriod;

protected:
    virtual void* poll() = 0;

public:

    Sensor(Time* time, long pollingPeriod);

    void* update();
    long getLastTimeRead();

    virtual size_t sensorDataBytes() const = 0;

};