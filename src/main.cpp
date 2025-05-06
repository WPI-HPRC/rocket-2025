#include <Arduino.h>

#include "Context.h"
#include "HardwareTimer.h"
#include "Wire.h"
#include "airbrakes/AirbrakeController.h"
#include "boilerplate/Sensors/Sensor/Sensor.h"
#include "pb.h"
#include "states/States.h"
#include <SPI.h>
#include <boilerplate/Sensors/SensorManager/SensorManager.h>
#include <boilerplate/StateMachine/StateMachine.h>

#include "config.h"

#include "telemetry/XBeeProSX.h"

#if defined(MARS)
SPIClass xbee_spi(XBEE_MOSI, XBEE_MISO, XBEE_SCLK);
#elif defined(POLARIS)
SPIClass xbee_spi = SPI;
#endif

Context ctx = {
#if defined(MARS)
    .accel = ASM330(),
    .baro = LPS22(),
    .mag = ICM20948(),
    .sd = SdFs(),
#elif defined(POLARIS)
    .accel = ICM42688_(),
    .baro = MS5611(),
    .mag = MMC5983(),
#endif
    .gps = MAX10S(),
    .airbrakes = AirbrakeController(AIRBRAKE_SERVO_PIN, AIRBRAKE_FEEDBACK_PIN),
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
bool state = true;

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

void loop10ms(); // Main update
void loop25ms(); // EKF
void loop250ms(); // Logging (not called when flightMode is set)
void loop1000ms(); // SD Flush

void timeHandler() {
    static uint64_t lastCall10ms = 0;
    static uint64_t lastCall1000ms = 0;

    bool run10ms = false;
    bool run1000ms = false;

    uint64_t now = micros();
    if (now - lastCall10ms >= 10000) {
        lastCall10ms = now;
        run10ms = true;
    }
    if (now - lastCall1000ms >= 1000000) {
        lastCall1000ms = now;
        run1000ms = true;
    }

    if (run10ms) {
        loop10ms();
    }
    if (run1000ms) {
        loop1000ms();
    }
}

void lowPrioTimeHandler() {
    static uint64_t lastCall25ms = 0;
    static uint64_t lastCall250ms = 0;

    bool run25ms = false;
    bool run250ms = false;

    uint64_t now = micros();
    if (now - lastCall25ms >= 25000) {
        lastCall25ms = now;
        run25ms = true;
    }
    if (!ctx.flightMode && now - lastCall250ms >= 250000) {
        lastCall250ms = now;
        run250ms = true;
    }

    if (run25ms) {
        loop25ms();
    }
    if (run250ms) {
        loop250ms();
    }
}

HardwareTimer loopTimer(TIM2);
HardwareTimer lowPrioTimer(TIM3);

void setup() {
#if defined(MARS)
    // P_Good pins
    pinMode(PE0, OUTPUT); // PG3V3_LED
    pinMode(PE1, OUTPUT); // PG5V_LED
    pinMode(PA3, INPUT);  // PG3V3
    pinMode(PC4, INPUT);  // PG5V

    digitalWrite(PE0, digitalRead(PA3));
    digitalWrite(PE1, digitalRead(PC4));
#endif
    Serial.begin(9600);

    ctx.airbrakes.init();

    Wire.setSCL(SENSOR_SCL);
    Wire.setSDA(SENSOR_SDA);
    Wire.begin();

#if defined(MARS)
    SPI.setSCLK(SD_SCLK);
#elif defined(POLARIS)
    SPI.setSCK(SD_SCLK);
#endif
    SPI.setMISO(SD_MISO);
    SPI.setMOSI(SD_MOSI);
    SPI.begin();

    // while (!Serial)
    //     delay(5);

    stateMachine.initialize();
    sensorManager.sensorInit();

    Wire.setClock(400000);

    pinMode(LED_PIN, OUTPUT);

#if defined(MARS)
    sd_initialized = ctx.sd.begin(SD_CS, SD_SPI_SPEED);
#elif defined(POLARIS)
    sd_initialized = SD.begin(SD_CS);
#endif

    if (sd_initialized) {
        int fileIdx = 0;
        char filename[100];
        while (fileIdx < 100) {
            sprintf(filename, "flightData%d.csv", fileIdx++);

            Serial.printf("Trying file `%s`\n", filename);
#if defined(MARS)
            if (!ctx.sd.exists(filename)) {
                ctx.logFile = ctx.sd.open(filename, O_RDWR | O_CREAT | O_TRUNC);
                break;
            }
#elif defined(POLARIS)
            if (!SD.exists(filename)) {
                ctx.logFile = SD.open(filename, FILE_WRITE_BEGIN);
                break;
            }
#endif
        }
    }

    if (ctx.logFile) {
        ctx.logCsvHeader();
    }

#if defined(MARS)
    xbee_spi.begin();
#endif

    xbee.start();

    loopTimer.setOverflow(10, MICROSEC_FORMAT);
    loopTimer.attachInterrupt(timeHandler);
    loopTimer.setInterruptPriority(10, 0);

    lowPrioTimer.setOverflow(500, MICROSEC_FORMAT);
    lowPrioTimer.attachInterrupt(lowPrioTimeHandler);
    lowPrioTimer.setInterruptPriority(11, 0);

    loopTimer.resume();
    lowPrioTimer.resume();
}

void loop10ms() {
#if defined(MARS)
    digitalWrite(PE0, digitalRead(PA3));
    digitalWrite(PE1, digitalRead(PC4));
#endif

    stateMachine.loop();
    sensorManager.loop();
    xbee.loop();

    if (sd_initialized && ctx.logFile) {
        ctx.logFile.print(millis());
        ctx.logFile.print(",");
        ctx.baro.logCsvRow(ctx.logFile);
        ctx.logFile.print(",");
        ctx.accel.logCsvRow(ctx.logFile);
        ctx.logFile.print(",");
        ctx.mag.logCsvRow(ctx.logFile);
        ctx.logFile.print(",");
        ctx.gps.logCsvRow(ctx.logFile);
        ctx.logFile.println();
    }
}

void loop25ms() {
}

void loop250ms() {
    ctx.accel.debugPrint(Serial);
    ctx.baro.debugPrint(Serial);
    ctx.gps.debugPrint(Serial);
    ctx.mag.debugPrint(Serial);

    if (sd_initialized && ctx.logFile) {
        state = !state;
    }
    digitalWrite(LED_PIN, state);    
}

void loop1000ms() {
    ctx.logFile.flush();
}

void loop() {}
