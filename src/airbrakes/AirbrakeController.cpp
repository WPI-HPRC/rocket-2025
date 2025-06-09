#include "AirbrakeController.h"

void AirbrakeController::init() {
  servo.attach(servo_pin);
  pinMode(feedback_pin, INPUT);
}

void AirbrakeController::write(int val) {
  servo.writeMicroseconds(val);
}

int AirbrakeController::read() {
  return analogRead(feedback_pin);
}

float AirbrakeController::deployAmmount(float acc_z, float v_z, float alt, float curr_deploy) {
  acc_z = (acc_z - 1) * g * -1;

  float curr_max_height = alt - (v_z*v_z)/(2*acc_z);
  if(curr_max_height <= target_height) {
    return 0;
  } else {
    float new_deploy = curr_deploy * 1.05;
    return (new_deploy >= 1 ? 1 : new_deploy); 
  }
}

float AirbrakeController::controllBreak() {
  const auto icmData = magData.getData();
  const auto accelData = accData.getData();
  const auto lpsData = baroData.getData();

  float acc_z_best = magData.accelZ;
  float alt_best = baroData.altitude;
  // populate in case no new data to use

  if(accelData.getLastUpdated() != lastAccRead) {
    acc_z_best = accelData.accelZ;
    lastAccRead = accelData.getLastUpdated();
  }

  // more accurate suposedly so better to use
  if(magData.getLastUpdated() != lastMagRead) {
    acc_z_best = magData.accelZ;
    lastMagRead = magData.getLastUpdated();
  }

  if(baroData.getLastUpdated() != lastMagRead) {
    alt_best = baroData.altitude;
    lastBaroRead = baroData.getLastUpdated();
  }

  return deployAmmount(acc_z_best, float v_z, alt_best, current_break_deploy);
  // still need to get v_z from somewhere
}
