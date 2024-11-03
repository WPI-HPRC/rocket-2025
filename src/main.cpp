#include "Rocket/Rocket.h"
#include "services/ArduinoTime.h"

#include <Arduino.h>

#include <Wire.h>

ArduinoTime timeService;
Rocket rocket(&timeService);

// Something in Rocket is causing issues

void setup() {
    Serial.begin(9600);

    Wire.begin(4000000);

    rocket.init();

}

void loop() {
    rocket.iterate();
    delay(5);
}
