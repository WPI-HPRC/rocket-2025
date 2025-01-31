#include "Rocket/Rocket.h"
#include <Arduino.h>
#include <cstring>

Rocket::Rocket(Time *time)
    : time(time), accelerometer(100, 0), sensorManager(SensorManager(time)) {}
// magnetometer(200, 1) {}

void Rocket::init() {
  sensorManager.addSensor(&accelerometer);
  // sensorManager.addSensor(&magnetometer);
  if (!sensorManager.sensorInit()) {
    Serial.println("sensorManager Failed");
  }
  if (!taskScheduler.add(&sensorManager)) {
    Serial.println("task scheduler failed to add SensorManager");
  }
}

void Rocket::iterate() { 
  taskScheduler.run();
}
