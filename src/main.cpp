#include <Arduino.h>
#include "services/ArduinoTime.h"
#include "Rocket/Rocket.h"

ArduinoTime timeService;
Rocket rocket(&timeService);

void setup() {
    Serial.begin(9600);

    rocket.iterate();

}

void loop() {
    rocket.iterate();
    delay(5);
}