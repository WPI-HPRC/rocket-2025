//
// Created by Daniel Coburn on 11/15/24.
//

#pragma once

#include <vector>
#include "Sensor.h"
/*
 * include information about sensors
 *  - hide it
 *  - allow adding sensors
 *  - allow for custom polling logic, so hardware is not going to be a big deal
 */

class SensorManager {
public:
    bool addSensor(Sensor* sensorPtr); // add sensor, true if added, false if not added
    void** readSensors(); // array of reading pointers

    // un-allocation
    virtual ~SensorManager() = default;

private:
    std::vector<Sensor*> sensors;

protected:
    //
};