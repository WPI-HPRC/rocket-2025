#pragma once

#include "../SensorManager/SensorManager.h"
#include "SensorManager/SensorManager.h"
#include "services/Time.h"
#include "TaskScheduler/TaskScheduler.h"

class Rocket {

private:
    Time *time;
    TaskScheduler taskScheduler;
    SensorManager sensorManager;

public:
    void iterate();
    void init();
    Rocket(Time* time);
};
