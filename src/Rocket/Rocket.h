#pragma once

#include "Sensor/Sensors.h"
#include "services/Time.h"

class Rocket {

private:
    Sensors sensors;
    Time *time;

public:
    void iterate();
    void init();
    Rocket(Time* time);
};
