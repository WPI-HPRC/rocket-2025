//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "services/Time.h"

#include <stdlib.h>

//template <class Data>
class Sensor {

protected:
    virtual void* poll() = 0;
    long lastTimeRead;
    long pollingPeriod;
    Time* time;

public:
    Sensor(Time* time, long pollingPeriod);

    virtual bool init() = 0;

    void* update();
    long getLastTimeRead();

    virtual size_t sensorDataBytes() const = 0;
    virtual ~Sensor() = default;
    /*
public:
    void* update();
    long getLastTimeRead();

    Sensor(Time* time, long pollingPeriod):
    time(time),
    pollingPeriod(pollingPeriod),
    lastTimeRead(time->millis())
    {}
    
    virtual bool init();

    virtual size_t sensorDataBytes() const = 0;
    virtual ~Sensor() = default;

    void* data; // what is this
     */
};