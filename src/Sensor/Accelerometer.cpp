//
// Created by Daniel Coburn on 10/31/24.
//

#include "Accelerometer.h"
#include <Wire.h>

bool Accelerometer::init() {

  if(icm42688.begin() != 1) return false;

  icm42688.setAccelFS(ICM42688::gpm16);
  icm42688.setGyroFS(ICM42688::dps250);
}


void* Accelerometer::poll() {
  icm42688.getAGT();

  data.accX = icm42688.accX();
  data.accY = icm42688.accY();
  data.accZ = icm42688.accZ();
  data.gyroX = icm42688.gyrX();
  data.gyroY = icm42688.gyrY();
  data.gyroZ = icm42688.gyrZ();
}

size_t Accelerometer::sensorDataBytes() const {
  return sizeof(Data);  
}
