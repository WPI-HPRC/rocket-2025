#include <Arduino.h>

#include "Context.h"
#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/ICM20948.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "boilerplate/Sensors/Impl/MAX10S.h"
#include "boilerplate/Sensors/Sensor/Sensor.h"
#include "Wire.h"
#include "states/States.h"
#include <boilerplate/Sensors/SensorManager/SensorManager.h>
#include <boilerplate/StateMachine/StateMachine.h>

// Context ctx = {
//     .accel = new ASM330(),
//     .baro = new Barometer(),
//     .icm = new ICM20948(),
//     .max10s = new MAX10S(),
// };

// Sensor *sensors[] = {ctx.accel, ctx.baro, ctx.icm};

// SensorManager<decltype(&millis), sizeof(sensors)/sizeof(Sensor*)> sensorManager(sensors, millis);

// StateMachine stateMachine((State *)new PreLaunch(&ctx));

void setup() {
    Serial.begin(9600);
    pinMode(PB9, OUTPUT);
    return;
    Wire.begin();
    // stateMachine.initialize();
    // sensorManager.sensorInit();
}

void loop() {
    digitalWrite(PB9, 1);
    return;
    // stateMachine.loop();
    // sensorManager.loop();
}
