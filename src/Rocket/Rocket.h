#pragma once

#include "../SensorManager/SensorManager.h"
#include "SensorManager/SensorManager.h"
#include "services/Time.h"
#include "TaskScheduler/TaskScheduler.h"
#include "../Sensor/Accelerometer/Accelerometer.h"
#include "../Sensor/Magnetometer/Magnetometer.h"

class Rocket {

private:
    Time *time;
    TaskScheduler taskScheduler;
    SensorManager sensorManager;
    Accelerometer accelerometer;
    //Magnetometer magnetometer;

public:
    void iterate();
    void init();
    Rocket(Time* time);
};
