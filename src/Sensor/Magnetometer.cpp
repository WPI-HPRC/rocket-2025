//
// Created by Daniel Coburn on 11/14/24.
//

#include "Magnetometer.h"

bool Magnetometer::init() {
    if (!this->mag.begin()) {
        return false;
    }

    this->mag.softReset();
    this->mag.setFilterBandwidth(800); // Filter Bandwith - BITS 0 | 0 (100Hz - 0.4 mG RMS Noise)

    return true;
}

void* Magnetometer::poll() {
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
}

size_t Magnetometer::sensorDataBytes() const {
    return sizeof(MagnetometerData);
}

MagnetometerData Magnetometer::getData() {
    return this->data;
}

void Magnetometer::debugData() {
    Serial.print("time: "); Serial.print(this->getData().id.timestamp);
    Serial.print(", x: "); Serial.print(this->data.x);
    Serial.print(", y: "); Serial.print(this->data.y);
    Serial.print(", z: "); Serial.print(this->data.x);
}