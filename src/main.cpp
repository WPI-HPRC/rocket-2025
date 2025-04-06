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
#include <SPI.h>
#include "SdFat.h"

SdFat sd;
File file;

#define SD_CS PA15
#define SD_SPI_SPEED SD_SCK_MHZ(4)

#define LED_PIN PB9

Context ctx = {
    .accel = new ASM330(),
    .baro = new Barometer(),
    .icm = new ICM20948(),
    .max10s = new MAX10S(),
};

Sensor *sensors[] = {ctx.accel, ctx.baro, ctx.icm, ctx.max10s};

SensorManager<decltype(&millis), sizeof(sensors)/sizeof(Sensor*)> sensorManager(sensors, millis);

StateMachine stateMachine((State *)new PreLaunch(&ctx));

bool sd_initialized = false;
long lastTime = 0;
bool state = true;

uint8_t error_code;

void setup() {
    Serial.begin(9600);

    // pinMode(PB9, OUTPUT);

    Wire.begin();

    stateMachine.initialize();
    sensorManager.sensorInit();

    // pinMode(LED_PIN, OUTPUT);

    // sd_initialized = sd.begin(SD_CS, SD_SPI_SPEED);
    // error_code = sd.card()->errorCode();

    lastTime = millis();
}

void loop() {
    stateMachine.loop();
    sensorManager.loop();

    // long now = millis();
    // if (sd_initialized && now - lastTime >= 1000) {
    //     lastTime = now;
    //     state = !state;
    // }
    // digitalWrite(LED_PIN, state);
    //
}

// Outputs the bits in the byte `data` in MSB order over `pin`
void output_byte(uint8_t data, uint pin) {
    digitalWrite(pin, 1);

    delay(50);
    digitalWrite(pin, 0);

    delay(50);

    digitalWrite(pin, 1);
    delay(50);
    digitalWrite(pin, 0);
    for (int bit = 0; bit < 8; bit++) {
        digitalWrite(pin, (data >> (7 - bit)) & 1);
        delay(100);
    }

    digitalWrite(pin, 0);

    delay(1000);
}
