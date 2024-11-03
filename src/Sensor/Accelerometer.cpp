//
// Created by Daniel Coburn on 10/31/24.
//

#include "Accelerometer.h"
#include <Wire.h>

bool Accelerometer::init() {
  Serial.println("Accelerometer init!");

  if(icm42688.begin() != 1) return false;

  icm42688.setAccelFS(ICM42688::gpm16);
  icm42688.setGyroFS(ICM42688::dps250);
  icm42688.setAccelODR(ICM42688::odr100);
  icm42688.setGyroODR(ICM42688::odr100);

  return true;
}


Data Accelerometer::poll() {
  // Serial.println("Accelerometer poll!");
  icm42688.getAGT();

  Data data = {
    .accX = icm42688.accX(),
    .accY = icm42688.accY(),
    .accZ = icm42688.accZ(),
    .gyroX = icm42688.gyrX(),
    .gyroY = icm42688.gyrY(),
    .gyroZ = icm42688.gyrZ(),
  };

  return data;
}

size_t Accelerometer::sensorDataBytes() const {
  return sizeof(Data);
}
