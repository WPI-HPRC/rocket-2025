//
// Created by Daniel Coburn on 9/27/24.
//

#ifndef NEW_BOARD_24_25_SENSOR_H
#define NEW_BOARD_24_25_SENSOR_H

#include <optional>

/*
Data = ptr + len_in_bytes
TimestampedData = timestamp + Data
Sensor:
    optional<Data> update()
    TimestampedData poll()
 */

struct Data {
    void* pointer;
    std::size_t len;
};

struct TimestampedData {
    Data data;
    long timestamp;
};

class Sensor {

protected:
    Data data;
    int lastTimeRead;

public:
    virtual std::optional<Data> update() = 0;
    virtual TimestampedData read();
};

#endif //NEW_BOARD_24_25_SENSOR_H
