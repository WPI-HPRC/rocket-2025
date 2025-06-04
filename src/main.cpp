#include <Arduino.h>

#include "Context.h"
#include "Wire.h"
#include "airbrakes/AirbrakeController.h"
#include "boilerplate/Looper/Looper.h"
#include "boilerplate/Sensors/Sensor/Sensor.h"
#include "boilerplate/StateEstimator/AttEkf.h"
#include "boilerplate/StateEstimator/PVKF.h"
#include "boilerplate/Utilities/SDSerialInterface.h"
#include "states/States.h"
#include <SPI.h>
#include <boilerplate/Sensors/SensorManager/SensorManager.h>
#include <boilerplate/StateMachine/StateMachine.h>

#include "config.h"

#include "telemetry/XBeeProSX.h"

#if defined(MARS)
SPIClass xbee_spi(XBEE_MOSI, XBEE_MISO, XBEE_SCLK);
SPIClass cam_spi(CAM_MOSI, CAM_MISO, CAM_SCK);
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
    .attEkfLogger = AttEkfLogger(),
    .pvKFLogger = PVEkfLogger(),
};

XbeeProSX xbee = XbeeProSX(&ctx, XBEE_CS, XBEE_ATTN, GROUNDSTATION_XBEE_ADDRESS,
                           &xbee_spi, 0);

Sensor *sensors[] = {&ctx.accel, &ctx.baro, &ctx.gps, &ctx.mag};

SensorManager sensorManager(sensors, millis);

StateMachine stateMachine((State *)new PreLaunch(&ctx));

AttStateEstimator quatEkf(ctx.mag.getData(), 0.025);
PVStateEstimator pvKF(ctx.baro.getData(), ctx.mag.getData(), ctx.gps.getData(), 0.025);
bool attEkfInitialized = false;
bool pvInitialized = false;

bool sd_initialized = false;

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

void mainLoop();       // Main update
void xbeeLoop();       // xbee send
void EKFLoop();        // EKF
void loggingLoop();    // Logging (not called when flightMode is set)
void occasionalLoop(); // For things like flushing SD card

Looper<FunctionsList<mainLoop, xbeeLoop, loggingLoop, occasionalLoop>,
       FunctionDelaysList<10u, 50u, 250u, 1000u>>
    looper(100, 10, TIM2);
Looper<FunctionsList<EKFLoop>, FunctionDelaysList<25u>> lowPrioLooper(1000, 11,
                                                                      TIM3);

void setup() {
#if defined(MARS)
    // P_Good pins
    pinMode(PE0, OUTPUT); // PG3V3_LED
    pinMode(PE1, OUTPUT); // PG5V_LED
    pinMode(PA3, INPUT);  // PG3V3
    pinMode(PC4, INPUT);  // PG5V

    digitalWrite(PE0, digitalRead(PA3));
    digitalWrite(PE1, digitalRead(PC4));

    pinMode(PC12, INPUT_PULLDOWN);

    if (digitalRead(PC12) == HIGH) {
        setupSDInterface(&ctx);
        return;
    }
#endif
    Serial.begin(9600);

    // idk if both of the `write`s are necessary, but it seems to help with it
    // not reseting to neutral for very long
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
    cam_spi.beginTransaction(SPISettings(328125, MSBFIRST, SPI_MODE0));

    pinMode(CAM_CS, OUTPUT);
    digitalWrite(CAM_CS, HIGH);

    looper.init();
    lowPrioLooper.init();
}

void mainLoop() {

    /*
    Serial.printf("BYTE: ");

    digitalWrite(CAM_CS, LOW);
    uint8_t *buff = new uint8_t;
    *buff = 77;
    cam_spi.transfer(buff, 1);
    Serial.printf("%d\n", *buff);
    digitalWrite(CAM_CS, HIGH);
*/

    static uint32_t lastBaroDataLogged = 0;
    static uint32_t lastAccelDataLogged = 0;
    static uint32_t lastMagDataLogged = 0;
    static uint32_t lastGpsDataLogged = 0;
    static uint32_t lastAttKfDataLogged = 0;
    static uint32_t lastPVKfDataLogged = 0;

#if defined(MARS)
    digitalWrite(PE0, digitalRead(PA3));
    digitalWrite(PE1, digitalRead(PC4));
#endif

    stateMachine.loop();
    sensorManager.loop();

    if (sd_initialized && ctx.logFile) {
        ctx.logFile.print(millis());
        ctx.logFile.print(",");

        lastBaroDataLogged = ctx.baro.logCsvRow(ctx.logFile, lastBaroDataLogged);
        ctx.logFile.print(",");

        lastAccelDataLogged = ctx.accel.logCsvRow(ctx.logFile, lastAccelDataLogged);
        ctx.logFile.print(",");

        lastMagDataLogged = ctx.mag.logCsvRow(ctx.logFile, lastMagDataLogged);
        ctx.logFile.print(",");

        lastGpsDataLogged = ctx.gps.logCsvRow(ctx.logFile, lastGpsDataLogged);
        ctx.logFile.print(",");

        lastAttKfDataLogged =
            ctx.attEkfLogger.logCsvRow(ctx.logFile, lastAttKfDataLogged);
        ctx.logFile.print(",");

        lastPVKfDataLogged =
            ctx.pvKFLogger.logCsvRow(ctx.logFile, lastPVKfDataLogged);
        ctx.logFile.println();
    }
}

void xbeeLoop() { xbee.loop(); }

void EKFLoop() {
    static TimedPointer<MAX10SData> gpsData = ctx.gps.getData();
    static TimedPointer<LPS22Data> baroData = ctx.baro.getData();

    if (attEkfInitialized && !pvInitialized &&
        (gpsData->gpsLockType == 3 || gpsData->gpsLockType == 2)) {
        BLA::Matrix<6, 1> initialPV = {(float)gpsData->lat, (float)gpsData->lon, baroData->altitude, 0, 0, 0};
        pvKF.init(initialPV, ctx.attEkfLogger.getState());
        pvInitialized = true;
    }

    if (!attEkfInitialized) {
        quatEkf.init();
        attEkfInitialized = true;
    }

    auto x = quatEkf.onLoop(stateMachine.getCurrentStateId() == ID_PreLaunch);

    if (pvInitialized) {
        auto pv = pvKF.onLoop();
        noInterrupts();
        ctx.pvKFLogger.newState(pv);
        interrupts();
    }

    // disabling interrupts here may not be necessary, but it guarantees we
    // don't read context from the high priority interrupt in an invalid state,
    // since that one can preempt this one.
    noInterrupts();
    ctx.attEkfLogger.newState(x);
    interrupts();
}

void loggingLoop() {
    static bool ledState = true;

    Serial.println(millis());
    Serial.println(ctx.gps.getInitStatus());
    ctx.accel.debugLog(Serial);
    ctx.baro.debugLog(Serial);
    ctx.gps.debugLog(Serial);
    ctx.mag.debugLog(Serial);
    ctx.attEkfLogger.debugLog(Serial);
    ctx.pvKFLogger.debugLog(Serial);

    if (sd_initialized && ctx.logFile) {
        ledState = !ledState;
    }
    digitalWrite(LED_PIN, ledState);
}

void occasionalLoop() { ctx.logFile.flush(); }

void loop() { handleSDInterface(&ctx); }
