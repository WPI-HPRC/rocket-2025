#include <Arduino.h>

#include "Context.h"
#include "Wire.h"
#include "airbrakes/AirbrakeController.h"
#include "boilerplate/Sensors/Sensor/Sensor.h"
#include "pb.h"
#include "states/States.h"
#include <SPI.h>
#include <boilerplate/Sensors/SensorManager/SensorManager.h>
#include <boilerplate/StateMachine/StateMachine.h>

#include <StateEstimator/attEKF/attEkf.h>

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
long lastTime = 0;
long lastTimeEkf = 0;
bool state = true;

bool ekfInitialized = false;

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

AttStateEstimator quatEkf = AttStateEstimator();

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
    error_code = ctx.sd.card()->errorCode();
#elif defined(POLARIS)
    sd_initialized = SD.begin(SD_CS);
    // error_code = SD.sdfs.card()->errorCode();
#endif

    // Serial.printf("%d %d\n", sd_initialized, error_code);

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

    lastTime = millis();
    lastFlush = millis();
}

BLA::Matrix<4,1> quatFromTwoVectors(BLA::Matrix<3,1> a, BLA::Matrix<3,1> b) {
    a = a / BLA::Norm(a);
    b = b / BLA::Norm(b);

    float dot_ab = (BLA::MatrixTranspose<BLA::Matrix<3,1>>(a) * b)(0);

    BLA::Matrix<3,1> cross_ab = {
        a(1)*b(2) - a(2)*b(1),
        a(2)*b(0) - a(0)*b(2),
        a(0)*b(1) - a(1)*b(0)
    };

    float w = sqrt((1 + dot_ab) * 0.5f);
    float f = 1.0f / (2.0f * w);
    BLA::Matrix<4,1> q = {
        w,
        cross_ab(0) * f,
        cross_ab(1) * f,
        cross_ab(2) * f
    };

    return q / BLA::Norm(q);
}


void loop() {
#if defined(MARS)
    digitalWrite(PE0, digitalRead(PA3));
    digitalWrite(PE1, digitalRead(PC4));
#endif
    stateMachine.loop();
    sensorManager.loop();
    xbee.loop();

    long now = millis();

    if(!ekfInitialized) {
        // Accelerometer
        BLA::Matrix<3,1> a_b = {
            ctx.mag.getData().accelX,
            ctx.mag.getData().accelY,
            ctx.mag.getData().accelZ
        };
        a_b = a_b / BLA::Norm(a_b);

        // Flip gravity direction: ensure z points DOWN in body frame
        float ax = a_b(0), ay = a_b(1), az = a_b(2);

        // Compute roll (phi) and pitch (theta) assuming NED frame
        float roll  = atan2(-ay, -az);  // Flip signs to match z-down NED
        float pitch = atan2(ax, sqrt(ay*ay + az*az));

        // Magnetometer
        BLA::Matrix<3,1> m_b = {
            ctx.mag.getData().magX,
            ctx.mag.getData().magY,
            ctx.mag.getData().magZ
        };

        // Tilt compensation
        float mx = m_b(0), my = m_b(1), mz = m_b(2);

        float mx2 = mx * cos(pitch) + mz * sin(pitch);
        float my2 = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

        float yaw = atan2(-my2, mx2);  // Heading angle (NED convention)

        // Convert roll/pitch/yaw to quaternion (ZYX convention)
        float cy = cos(yaw * 0.5f);
        float sy = sin(yaw * 0.5f);
        float cp = cos(pitch * 0.5f);
        float sp = sin(pitch * 0.5f);
        float cr = cos(roll * 0.5f);
        float sr = sin(roll * 0.5f);

        BLA::Matrix<4,1> q0 = {
            cr * cp * cy + sr * sp * sy,  // w
            sr * cp * cy - cr * sp * sy,  // x
            cr * sp * cy + sr * cp * sy,  // y
            cr * cp * sy - sr * sp * cy   // z
        };

        BLA::Matrix<13,1> x0 = {q0(0), q0(1), q0(2), q0(3),
            0, 0, 0,   // gyro bias
            0, 0, 0,   // accel bias
            0, 0, 0};  // mag bias

        quatEkf.init(x0, 0.025);
        ekfInitialized = true;
    }


    if(now - lastTimeEkf >= 25) {
        quatEkf.onLoop(ctx);
        
        lastTimeEkf = now;
    }

    if (now - lastTime >= 100) {
        lastTime = now;

        // Serial.print("Time: "); Serial.print(millis()); Serial.print(", ");
        // ctx.accel.debugPrint(Serial);
        // ctx.baro.debugPrint(Serial);
        // ctx.gps.debugPrint(Serial);
        // ctx.mag.debugPrint(Serial);
        // Serial.print("Flight mode: "); Serial.println(ctx.flightMode);

        BLA::Matrix<13,1> xq = ctx.quatState;

        BLA::Matrix<13,13> P = ctx.P;

        Serial.println("<----- State ----->");
        for (int i = 0; i < xq.Rows; i++) {
            Serial.print(xq(i, 0), 4);  // 4 decimal places
            Serial.print("\t");
        }
        Serial.println("");

        Serial.println("<----- Error Covariance ----->");
        for (int i = 0; i < P.Rows; i++) {
            for (int j = 0; j < P.Cols; j++) {
                Serial.print(P(i, j), 4);  // 4 decimal places
                Serial.print("\t");
            }
            Serial.println("");
        }

        // Serial CSV
        Serial.println("");
        Serial.print(millis());
        Serial.print(",");
        ctx.baro.logCsvRow(Serial);
        Serial.print(",");
        ctx.accel.logCsvRow(Serial);
        Serial.print(",");
        ctx.mag.logCsvRow(Serial);
        Serial.print(",");
        ctx.gps.logCsvRow(Serial);
        Serial.println();

        if (sd_initialized && ctx.logFile) {
            state = !state;
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
    digitalWrite(LED_PIN, state);

    if (now - lastFlush >= 1000) {
        lastFlush = now;
        ctx.logFile.flush();
    }

    ctx.vehicleState = stateMachine.getCurrentStateId();

    delay(1);
}