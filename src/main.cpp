#include "Rocket/Rocket.h"
#include "services/ArduinoTime.h"

#include <Arduino.h>

ArduinoTime timeService;
//Rocket rocket(&timeService);

// Something in Rocket is causing issues

void setup() {
    Serial.begin(9600);

    //rocket.iterate();

}

void loop() {
    Serial.print("ajfdjfd");
    //rocket.iterate();
    delay(5);
}
