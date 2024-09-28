#pragma once

#include "Sensor/Sensors.h"

class Rocket {

private:
    Sensors sensors;

public:
    void iterate();

};