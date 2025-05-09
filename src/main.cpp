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

    // Intialize Quaternion
    // BLA::Matrix<13,1> xq_0 = {0.8776, 0, 0.4794, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

    // quatEkf.init(xq_0, 0.01);
}

// BLA::Matrix<4,1> quatFromTwoVectors(BLA::Matrix<3,1> a, BLA::Matrix<3,1> b) {
//     a = a / BLA::Norm(a);
//     b = b / BLA::Norm(b);

//     float dot_ab = (BLA::MatrixTranspose<BLA::Matrix<3,1>>(a) * b)(0);

//     BLA::Matrix<3,1> cross_ab = {
//         a(1)*b(2) - a(2)*b(1),
//         a(2)*b(0) - a(0)*b(2),
//         a(0)*b(1) - a(1)*b(0)
//     };

//     float w = sqrt((1 + dot_ab) * 0.5f);
//     float f = 1.0f / (2.0f * w);
//     BLA::Matrix<4,1> q = {
//         w,
//         cross_ab(0) * f,
//         cross_ab(1) * f,
//         cross_ab(2) * f
//     };

//     return q / BLA::Norm(q);
// }

BLA::Matrix<3,1> cross(const BLA::Matrix<3,1>& a, const BLA::Matrix<3,1>& b) {
    return {
        a(1)*b(2) - a(2)*b(1),
        a(2)*b(0) - a(0)*b(2),
        a(0)*b(1) - a(1)*b(0)
    };
}

BLA::Matrix<4,1> rot2quat(const BLA::Matrix<3,3>& R) {
    float trace = R(0,0) + R(1,1) + R(2,2);
    BLA::Matrix<4,1> q;

    if (trace > 0.0f) {
        float s = sqrt(trace + 1.0f) * 2.0f;
        q(0) = 0.25f * s;
        q(1) = (R(2,1) - R(1,2)) / s;
        q(2) = (R(0,2) - R(2,0)) / s;
        q(3) = (R(1,0) - R(0,1)) / s;
    } else {
        if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
            float s = sqrt(1.0f + R(0,0) - R(1,1) - R(2,2)) * 2.0f;
            q(0) = (R(2,1) - R(1,2)) / s;
            q(1) = 0.25f * s;
            q(2) = (R(0,1) + R(1,0)) / s;
            q(3) = (R(0,2) + R(2,0)) / s;
        } else if (R(1,1) > R(2,2)) {
            float s = sqrt(1.0f + R(1,1) - R(0,0) - R(2,2)) * 2.0f;
            q(0) = (R(0,2) - R(2,0)) / s;
            q(1) = (R(0,1) + R(1,0)) / s;
            q(2) = 0.25f * s;
            q(3) = (R(1,2) + R(2,1)) / s;
        } else {
            float s = sqrt(1.0f + R(2,2) - R(0,0) - R(1,1)) * 2.0f;
            q(0) = (R(1,0) - R(0,1)) / s;
            q(1) = (R(0,2) + R(2,0)) / s;
            q(2) = (R(1,2) + R(2,1)) / s;
            q(3) = 0.25f * s;
        }
    }

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

    // if(!ekfInitialized) {
    //     BLA::Matrix<3,1> a_b = {
    //         ctx.mag.getData().accelX,
    //         ctx.mag.getData().accelY,
    //         ctx.mag.getData().accelZ
    //     };

    //     a_b = a_b / BLA::Norm(a_b);

    //     BLA::Matrix<3,1> z_ned = {0, 0, -9.80665};

    //     BLA::Matrix<4,1> q0 = quatFromTwoVectors(a_b, z_ned);

    //     BLA::Matrix<13,1> x0 = {q0(0), q0(1), q0(2), q0(3),
    //         0, 0, 0,   // gyro bias
    //         0, 0, 0,   // accel bias
    //         0, 0, 0};  // mag bias

    //     quatEkf.init(x0, 0.025);

    //     ekfInitialized = true;
    // }
    if (!ekfInitialized) {
        // === 1. Get Accelerometer Reading (Gravity Vector) ===
        BLA::Matrix<3,1> a_b = {
            ctx.mag.getData().accelX,
            ctx.mag.getData().accelY,
            ctx.mag.getData().accelZ
        };
        a_b = a_b / BLA::Norm(a_b);  // normalize gravity vector
    
        // === 2. Get Magnetometer Reading (Magnetic Field Vector) ===
        BLA::Matrix<3,1> m_b = {
            ctx.mag.getData().magX,
            ctx.mag.getData().magY,
            ctx.mag.getData().magZ
        };
        m_b = m_b / BLA::Norm(m_b);  // normalize magnetic field vector
    
        // === 3. Define Reference Vectors in NED Frame ===
        BLA::Matrix<3,1> z_ned = {0, 0, -1};       // Down
        BLA::Matrix<3,1> x_ned = {1, 0, 0};        // Magnetic North
    
        // === 4. Compute Body X and Y using Gram-Schmidt ===
        BLA::Matrix<3,1> y_b = cross(a_b, m_b);
        y_b = y_b / BLA::Norm(y_b);
    
        BLA::Matrix<3,1> x_b = cross(y_b, a_b);
    
        // Rotation matrix from NED to Body
        BLA::Matrix<3,3> R_bn = {
            x_b(0), y_b(0), a_b(0),
            x_b(1), y_b(1), a_b(1),
            x_b(2), y_b(2), a_b(2)
        };
    
        BLA::Matrix<4,1> q0 = rot2quat(R_bn);  // You need this function
    
        BLA::Matrix<13,1> x0 = {
            q0(0), q0(1), q0(2), q0(3),
            0, 0, 0,   // gyro bias
            0, 0, 0,   // accel bias
            0, 0, 0    // mag bias
        };
    
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