#pragma once

#include "BasicLinearAlgebra.h"
#include "airbrakes/AirbrakeController.h"
#include "config.h"

struct Context {
#if defined(MARS)
    ASM330 accel;
    LPS22 baro;
    ICM20948 mag;
    SdFs sd;
#elif defined(POLARIS)
    ICM42688_ accel;
    MS5611 baro;
    MMC5983 mag;
#endif
    MAX10S gps;
    AirbrakeController airbrakes;
    File logFile;
    bool flightMode;
    BLA::Matrix<13, 1> quatState;
    BLA::Matrix<6,1> pvState; 

    void logCsvHeader() {
        logFile.print("timestamp,");
        baro.logCsvHeader(logFile);
        logFile.print(",");
        accel.logCsvHeader(logFile);
        logFile.print(",");
        mag.logCsvHeader(logFile);
        logFile.print(",");
        gps.logCsvHeader(logFile);
        logFile.println();
    }
};
