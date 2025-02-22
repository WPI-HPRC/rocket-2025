#include <Arduino.h>

#include "Context.h"
#include "sensors/LPS25.h"
#include "states/States.h"
#include <boilerplate/Sensors/SensorManager/SensorManager.h>
#include <boilerplate/StateMachine/StateMachine.h>

Context ctx = {
    .baro = new Barometer(),
};

Sensor *sensors[1] = {ctx.baro};
SensorManager<decltype(&millis), 1> sensorManager(sensors, millis);

StateMachine stateMachine((State *)new PreLaunch(&ctx));

void setup() {
    Serial.begin(9600);
    Wire.begin();
    stateMachine.initialize();
    sensorManager.sensorInit();
}

void loop() {
    stateMachine.loop();
    sensorManager.loop();
}
