//
// Created by Daniel Coburn on 11/14/24.
//

#include "Magnetometer.h"

/**
 * public \n
 * initialize sensor
 * @return true if the sensor was initialized correctly, false if not
 */
bool Magnetometer::init() {
    if (!this->mag.begin()) {
        return false;
    }

    this->mag.softReset();
    this->mag.setFilterBandwidth(800); // Filter Bandwidth - BITS 0 | 0 (100Hz - 0.4 mG RMS Noise)

    initStatus = true;
    return true;
}

/**
 * protected \n
 * read data from the Magnetometer
 * @return void*, contains data from the sensor
 */
void* Magnetometer::poll() {
    if(initStatus) {
        uint32_t rawX = mag.getMeasurementX();
        uint32_t rawY = mag.getMeasurementY();
        uint32_t rawZ = mag.getMeasurementZ();

        // Convert raw 18 bit unsigned integer to +/- 1.0 approximate zero is 2^17 (131072)
        double scaledX = (double) rawX - 131072.0;
        scaledX /= 131072.0;
        double scaledY = (double) rawY - 131072.0;
        scaledY /= 131072.0;
        double scaledZ = (double) rawZ - 131072.0;
        scaledZ /= 131072.0;

        data.x = scaledX * 800000.0; // [nT]
        data.y = scaledY * 800000.0; // [nT]
        data.z = scaledZ * 800000.0; // [nT]

        return &data;
    } else {
        return nullptr;
    }
}

/**
 * protected \n
 * gets size of data in bytes
 * @return size of data in bytes
 */
size_t Magnetometer::sensorDataBytes() const {
    return sizeof(MagnetometerData);
}

/**
 * public \n
 * @return std::optional, data the sensor got, or std::nullopt if no sensor
 */
std::optional<MagnetometerData> Magnetometer::getData() {
    if(initStatus) {
        return data;
    } else {
        return std::nullopt;
    }
}

/**
 * public \n
 * prints data values to screen, good for debug
 */
void Magnetometer::debugData() {
    if(initStatus) {
        Serial.print("time: "); Serial.print(this->data.id.timestamp);
        Serial.print(", x: "); Serial.print(this->data.x);
        Serial.print(", y: "); Serial.print(this->data.y);
        Serial.print(", z: "); Serial.println(this->data.x);
    } else {
        Serial.println("No Magnetometer");
    }
}