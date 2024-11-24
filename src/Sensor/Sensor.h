//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "services/Time.h"
#include <stdlib.h>

struct ID {
    long timestamp;
    // int sensorNumber;
    // would having a field for telemetry use here be useful??
};

class Sensor {

protected:
    long lastTimeRead;
    long pollingPeriod;
    //Time* time;
    virtual void* poll() = 0;
    bool initStatus = false;
    int sensorNumber;

private:
    ID id{};

public:
    Sensor(long pollingPeriod, int sensorNumber) :
        sensorNumber(sensorNumber),
        pollingPeriod(pollingPeriod),
        lastTimeRead(0) { }

    bool getInitStatus();
    virtual bool init() = 0;
    virtual void debugData() = 0;
    void* update(long currentTime);
    long getLastTimeRead();
    long getPollingPeriod();
    virtual size_t sensorDataBytes() const = 0;
    virtual ~Sensor() = default;
};