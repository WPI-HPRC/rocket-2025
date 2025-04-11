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
#define SD_SPI_SPEED SD_SCK_MHZ(50)

#define LED_PIN PB9

Context ctx = {
    .accel = new ASM330(),
    .baro = new Barometer(),
    .icm = new ICM20948(),
    .max10s = new MAX10S(),
};

Sensor *sensors[] = {ctx.accel, ctx.baro, ctx.max10s};
SensorManager<decltype(&millis), sizeof(sensors)/sizeof(Sensor*)> sensorManager(sensors, millis);

StateMachine stateMachine((State *)new PreLaunch(&ctx));

bool sd_initialized = false;
long lastTime = 0;
bool state = true;

long lastFlush = 0;

uint8_t error_code;

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

void setup() {
    Serial.begin(9600);

    pinMode(PB9, OUTPUT);

    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();
    SPI.setSCLK(PB3);
    SPI.setMISO(PB4);
    SPI.setMOSI(PB5);
    SPI.begin();

    stateMachine.initialize();
    sensorManager.sensorInit();

    Wire.setClock(400000);

    pinMode(LED_PIN, OUTPUT);

    sd_initialized = sd.begin(SD_CS, SD_SPI_SPEED);
    error_code = sd.card()->errorCode();

    lastTime = millis();

    file = sd.open("test.txt", O_RDWR | O_CREAT | O_TRUNC);

    lastFlush = millis();
}

void loop() {
    stateMachine.loop();
    sensorManager.loop();

    ctx.accel->debugPrint(file);
    ctx.baro->debugPrint(file);
    ctx.max10s->debugPrint(file);

    file.println(millis());

    long now = millis();
    if (sd_initialized && now - lastTime >= 250) {
        lastTime = now;
        state = !state;
    }
    digitalWrite(LED_PIN, state);

    if (now - lastFlush >= 3000) {
        lastFlush = now;
        file.flush();
    }
}

