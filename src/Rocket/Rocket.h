#pragma once

#include "Sensor/SensorManager.h"
#include "services/Time.h"

class Rocket {

private:
    SensorManager sensorManager;
    Time *time;

public:
    void iterate();
    void init();
    Rocket(Time* time);
};
