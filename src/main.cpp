#include "Rocket/Rocket.h"
#include "services/ArduinoTime.h"
#include <Arduino.h>
#include <Wire.h>

ArduinoTime timeService = {};
Rocket rocket(reinterpret_cast<Time *>(&timeService));

void setup() {
    Serial.begin(9600);
    Wire.begin(4000000);
    delay(5);
    Serial.println("rocket init start...");
    rocket.init();
}

void loop() {
    rocket.iterate();
    delay(5);
}
