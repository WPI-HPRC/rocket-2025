#pragma once

#include "BasicLinearAlgebra.h"
#include "airbrakes/AirbrakeController.h"
#include "boilerplate/StateEstimator/AttEkf.h"
#include "boilerplate/StateEstimator/PVKF.h"
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
    File errorLogFile;
    bool flightMode;
    uint32_t xbeeLoggingDelay;
    AttEkfLogger attEkfLogger;
    PVEkfLogger pvKFLogger;
    float initialAltitude;

    void logCsvHeader() {
        logFile.print("timestamp,state,flightMode,");
        baro.logCsvHeader(logFile);
        logFile.print(",");
        accel.logCsvHeader(logFile);
        logFile.print(",");
        mag.logCsvHeader(logFile);
        logFile.print(",");
        gps.logCsvHeader(logFile);
        logFile.print(",");
        attEkfLogger.logCsvHeader(logFile);
        logFile.print(",");
        pvKFLogger.logCsvHeader(logFile);
        logFile.print(",airbrakeServo");
        logFile.println();
    }
};
