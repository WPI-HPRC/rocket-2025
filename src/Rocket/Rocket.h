#pragma once

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

#include "Sensor/Sensor.h"
#include "Sensor/ExampleSensor.h"
#include "services/Time.h"

#define SENSORS(X)                                                             \
  X(ExampleSensor, s1, 1000)                                                   \
  X(ExampleSensor, s2, 5000)

CREATE_SENSORS(SENSORS)

struct Rocket {

  Sensors sensors;
  Time *time;

  void iterate() {
    uint8_t buf[sensors.largestBufferSize() + 1];
    // Use the first index in the buffer as the bitfield
    buf[0] = 0;

    size_t n = 1 + sensors.update(buf, time->millis(), buf + 1, sizeof buf - 1);

    if (n > 1) {
      for (int i = 0; i < n; i++) {
        Serial.printf("%d", buf[i]);
        if (i + 1 < n) {
          Serial.print(" ");
        }
      }
      Serial.println("\n------------------");
    }
  }

  Rocket(Time *time) : time(time) {}
};
