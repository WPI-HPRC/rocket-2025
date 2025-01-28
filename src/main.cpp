#include <Arduino.h>
#include "boilerplate/Sensors/SensorManager/SensorManager.h"

SensorManager sensorManager;

void setup() {
    Serial.begin(9600);
}

void loop() {
    delay(5);
    sensorManager.run();
}
