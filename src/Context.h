#pragma once

#include "airbrakes/AirbrakeController.h"
#include "config.h"

struct Context {
#if defined(MARS)
    ASM330 accel;
    LPS22 baro;
    ICM20948 mag;
#elif defined(POLARIS)
    ICM42688_ accel;
    MS5611 baro;
    MMC5983 mag;
#endif
    SdFs sd;
    MAX10S gps;
    AirbrakeController airbrakes;
    FsFile logFile;
    bool flightMode;

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
