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

#include "pb_decode.h"
#include "pb_encode.h"
#include "RocketTelemetryPacket.pb.h"
#include "telemetry/XBeeProSX.h"

#if defined(MARS)
SdFat sd;
#endif
File file;

#define SD_SPI_SPEED SD_SCK_MHZ(50)

HPRC_RocketTelemetryPacket packet = HPRC_RocketTelemetryPacket_init_zero;
uint8_t buffer[4096];
pb_ostream_t ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));

#if defined(MARS)
SPIClass xbee_spi;
#elif defined(POLARIS)
SPIClass xbee_spi = SPI;
#endif

XbeeProSX xbee(XBEE_CS, &xbee_spi);

Context ctx = {
#if defined(MARS)
    .accel = new ASM330(),
    .baro = new LPS22(),
    .mag = new ICM20948(),
#elif defined(POLARIS)
    .accel = new ICM42688_(),
    .baro = new MS5611(),
    .mag = new MMC5983(),
#endif
    .gps = new MAX10S(),
};

#if defined(MARS)
Sensor *sensors[] = {ctx.accel, ctx.baro, ctx.gps};
#elif defined(POLARIS)
Sensor *sensors[] = {ctx.accel, ctx.baro, ctx.mag, ctx.gps};
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
    xbee_spi.setSCLK(XBEE_SCLK);
    xbee_spi.setMISO(XBEE_MISO);
    xbee_spi.setMOSI(XBEE_MOSI);
    xbee_spi.begin();

    SPI.setSCLK(SD_SCLK);
#elif defined(POLARIS)
    SPI.setSCK(SD_SCLK);
#endif
    SPI.setMISO(SD_MISO);
    SPI.setMOSI(SD_MOSI);
    SPI.begin();

    stateMachine.initialize();
    sensorManager.sensorInit();

    Wire.setClock(400000);

    pinMode(LED_PIN, OUTPUT);

    xbee.start();

#if defined(MARS)
    sd_initialized = sd.begin(SD_CS, SD_SPI_SPEED);
    error_code = sd.card()->errorCode();

    file = sd.open("test.txt", O_RDWR | O_CREAT | O_TRUNC);
#elif defined(POLARIS)
    sd_initialized = SD.begin(SD_CS);

    file = SD.open("test.txt", FILE_WRITE_BEGIN);
#endif

    lastTime = millis();
    lastFlush = millis();
}

void loop() {
    stateMachine.loop();
    sensorManager.loop();

    // ctx.accel->debugPrint(file);
    // ctx.baro->debugPrint(file);
    // ctx.gps->debugPrint(Serial);

    // file.println(millis());

    long now = millis();
    if (sd_initialized && now - lastTime >= 250) {
        lastTime = now;
        state = !state;
    }
    digitalWrite(LED_PIN, state);

    if (now - lastFlush >= 3000) {
        packet.timestamp = now;
        packet.altitude = ctx.baro->getData().altitude;
        pb_encode(&ostream, &HPRC_RocketTelemetryPacket_msg, &packet);
        xbee_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        xbee.sendTransmitRequestCommand(0x0013A200423F474C, buffer, ostream.bytes_written);
        xbee_spi.endTransaction();

        lastFlush = now;
        file.flush();
    }
}
