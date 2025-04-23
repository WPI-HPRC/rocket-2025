#pragma once

#include "airbrakes/AirbrakeController.h"
#include "config.h"

struct Context {
#if defined(MARS)
    ASM330 accel;
    LPS22 baro;
    ICM20948 mag;
    SdFat sd;
#elif defined(POLARIS)
    ICM42688_ accel;
    MS5611 baro;
    MMC5983 mag;
#endif
    MAX10S gps;
    AirbrakeController airbrakes;
    File logFile;
    bool flightMode;
};
