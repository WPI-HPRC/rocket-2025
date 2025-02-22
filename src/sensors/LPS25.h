#pragma once

#include "../boilerplate/Sensors/Sensor/Sensor.h"
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>

struct BarameterData {
  float pressure;
};

class Barameter : public Sensor {
  public:
    Barameter() : Sensor(sizeof(BarameterData), 40), lps() {}

    BarameterData getData() {
      return *(BarameterData *)data;
    }

  private:
    Adafruit_LPS25 lps;

    bool init_impl() override {
      if (!lps.begin_I2C(0x5c)) {
        return false;
      }
      lps.setDataRate(LPS25_RATE_25_HZ);
      return true;
    }

    void *poll() override {
      sensors_event_t pressure;
      lps.getEvent(&pressure, nullptr);
      ((BarameterData *)data)->pressure = pressure.pressure;
      return data;
    }
};
