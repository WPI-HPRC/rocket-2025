#pragma once

#include "airbrakes/AirbrakeController.h"
#include "config.h"
#include "BasicLinearAlgebra.h"

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

    float q_w;
    float q_x;
    float q_y;
    float q_z;
    
    float q_cov_w;
    float q_cov_x;
    float q_cov_y;
    float q_cov_z;

    /*
    telem_packet->covQW = ctx->quatState(0);
        telem_packet->covQX = ctx->quatState(1);
        telem_packet->covQY = ctx->quatState(2);
        telem_packet->covQZ = ctx->quatState(3);*/

    BLA::Matrix<13,1> quatState;
    BLA::Matrix<13,13> P;

    int vehicleState = 0;

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