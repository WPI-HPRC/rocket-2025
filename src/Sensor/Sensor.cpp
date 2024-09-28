//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

TimestampedData Sensor::read() {
    return TimestampedData {
            .data = this->data,
            .timestamp = this->lastTimeRead
    };
}