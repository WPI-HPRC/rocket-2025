//
// Created by Daniel Coburn on 10/31/24.
//

#include "Accelerometer.h"
#include <Wire.h>

/**
 * public \n
 * initialize the sensor
 * @return true if all went well, false if not
 */
bool Accelerometer::init() {
    Serial.println("init for acc");
    if(icm42688.begin() != 1) {
        Serial.println("init for acc failed");
        return false;
    }

    icm42688.setAccelFS(ICM42688::gpm16);
    icm42688.setGyroFS(ICM42688::dps250);
    icm42688.setAccelODR(ICM42688::odr100);
    icm42688.setGyroODR(ICM42688::odr100);

    Serial.println("init for acc did not fail");
    initStatus = true;
    return true;
}

/**
 * protected \n
 * read data from sensor
 * @return void*, contains data from Accelerometer
 */
void* Accelerometer::poll() {
    Serial.println("yes i am being polled");
    if(initStatus) {
        icm42688.getAGT();

        data.accX = icm42688.accX();
        data.accY = icm42688.accY();
        data.accZ = icm42688.accZ();
        data.gyroX = icm42688.gyrX();
        data.gyroY = icm42688.gyrY();
        data.gyroZ = icm42688.gyrZ();

        return &data;
    } else {
        return nullptr;
    }
}

/**
 * protected \n
 * gets size of data
 * @return size of Accelerometer data in bytes
 */
size_t Accelerometer::sensorDataBytes() const {
  return sizeof(AccelerometerData);
}

/**
 * public \n
 * gets data, or std::nullopt if no Accelerometer
 * @return data, or std::nullopt if no sensor
 */
std::optional<AccelerometerData> Accelerometer::getData() {
    if(initStatus) {
        return data;
    } else {
        return std::nullopt;
    }
}

/**
 * public \n
 * good for debug, prints data to screen
 */
void Accelerometer::debugData() {
    if(initStatus) {
        Serial.print("time: "); Serial.print(this->data.id.timestamp);
        Serial.print(", accX: "); Serial.print(this->data.accX);
        Serial.print(", accY: "); Serial.print(this->data.accY);
        Serial.print(", accZ: "); Serial.print(this->data.accZ);
        Serial.print(", gyroX: "); Serial.print(this->data.gyroX);
        Serial.print(", gyroY: "); Serial.print(this->data.gyroY);
        Serial.print(", gyroZ: "); Serial.println(this->data.gyroZ);
    } else {
        Serial.println("No Accelerometer");
    }
}
