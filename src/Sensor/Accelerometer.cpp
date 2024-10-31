//
// Created by Daniel Coburn on 10/31/24.
//

#include "Accelerometer.h"
#include <Wire.h>
const int addr = 0x68;

// bool Accelerometer::init(Context *ctx) {

//     ctx->icm42688(Wire, addr); // has issues
//     // no match for call to '(ICM42688) (TwoWire&, const int&)'

//     if(ctx->icm42688.begin() != 1)
//         return false;

//     ctx->icm42688.setAccelFS(ICM42688::gpm16);
//     ctx->icm42688.setGyroFS(ICM42688::dps250);

//     ctx->icm42688.setAccelODR(ICM42688::odr100);
//     ctx->icm42688.setGyroODR(ICM42688::odr100);

//     return true;
// }

// Accelerometer() {
//   if(icm42688.begin() != 1) return;

//   icm42688.setAccelFS(ICM42688::gpm16);
//   icm42688.setGyroFS(ICM42688::dps250);

//   icm42688.setAccelODR(ICM42688::odr100);
//   icm42688.setGyroODR(ICM42688::odr100);
// }



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
