#include <Arduino.h>

#include "Context.h"
#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/ICM20948.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "states/States.h"
#include <boilerplate/Sensors/SensorManager/SensorManager.h>
#include <boilerplate/StateMachine/StateMachine.h>

Context ctx = {
    .accel = new ASM330(),
    .baro = new Barometer(),
    .icm = new ICM20948(),
};

Sensor *sensors[3] = {ctx.accel, ctx.baro, ctx.icm};

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
