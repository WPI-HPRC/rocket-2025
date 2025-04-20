#include <Arduino.h>

#include "Context.h"
#include "Wire.h"
#include "boilerplate/Sensors/Sensor/Sensor.h"
#include "pb.h"
#include "states/States.h"
#include <SPI.h>
#include <boilerplate/Sensors/SensorManager/SensorManager.h>
#include <boilerplate/StateMachine/StateMachine.h>

#include "config.h"

#include "telemetry/XBeeProSX.h"

#if defined(MARS)
SdFat sd;
#endif

#define SD_SPI_SPEED SD_SCK_MHZ(50)

#if defined(MARS)
SPIClass xbee_spi;
#elif defined(POLARIS)
SPIClass xbee_spi = SPI;
#endif

Context ctx = {
#if defined(MARS)
    .accel = ASM330(),
    .baro = LPS22(),
    .mag = ICM20948(),
#elif defined(POLARIS)
    .accel = ICM42688_(),
    .baro = MS5611(),
    .mag = MMC5983(),
#endif
    .gps = MAX10S(),
    .flightMode = false,
};

XbeeProSX xbee =
    XbeeProSX(&ctx, XBEE_CS, XBEE_ATTN, GROUNDSTATION_XBEE_ADDRESS, &xbee_spi);

#if defined(MARS)
Sensor *sensors[] = {&ctx.accel, &ctx.baro, &ctx.gps, &ctx.mag};
#elif defined(POLARIS)
Sensor *sensors[] = {&ctx.accel, &ctx.baro, &ctx.mag, &ctx.gps};
#endif

SensorManager<decltype(&millis), sizeof(sensors) / sizeof(Sensor *)>
    sensorManager(sensors, millis);

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

    Wire.setSCL(SENSOR_SCL);
    Wire.setSDA(SENSOR_SDA);
    Wire.begin();

#if defined(MARS)
    // xbee_spi.setSCLK(XBEE_SCLK);
    // xbee_spi.setMISO(XBEE_MISO);
    // xbee_spi.setMOSI(XBEE_MOSI);
    // xbee_spi.begin();

    SPI.setSCLK(SD_SCLK);
#elif defined(POLARIS)
    SPI.setSCK(SD_SCLK);
#endif
    SPI.setMISO(SD_MISO);
    SPI.setMOSI(SD_MOSI);
    SPI.begin();

    // while (!Serial) delay(5);

    stateMachine.initialize();
    sensorManager.sensorInit();

    Wire.setClock(400000);

    pinMode(LED_PIN, OUTPUT);

    xbee.start();

#if defined(MARS)
    sd_initialized = sd.begin(SD_CS, SD_SPI_SPEED);
    error_code = sd.card()->errorCode();

    ctx.logFile = sd.open("test.txt", O_RDWR | O_CREAT | O_TRUNC);
#elif defined(POLARIS)
    sd_initialized = SD.begin(SD_CS);

    ctx.logFile = SD.open("test.txt", FILE_WRITE_BEGIN);
#endif

    lastTime = millis();
    lastFlush = millis();
}

void loop() {
    stateMachine.loop();
    sensorManager.loop();
    xbee.loop();

    ctx.accel.debugPrint(ctx.logFile);
    ctx.baro.debugPrint(ctx.logFile);
    ctx.gps.debugPrint(ctx.logFile);
    ctx.mag.debugPrint(ctx.logFile);
    ctx.accel.debugPrint(Serial);
    ctx.baro.debugPrint(Serial);
    ctx.gps.debugPrint(Serial);
    ctx.mag.debugPrint(Serial);

    long now = millis();
    if (sd_initialized && now - lastTime >= 250) {
        lastTime = now;
        state = !state;
    }
    digitalWrite(LED_PIN, state);

    if (now - lastFlush >= 50) {
        lastFlush = now;
        ctx.logFile.flush();
    }

    delay(10);
}
