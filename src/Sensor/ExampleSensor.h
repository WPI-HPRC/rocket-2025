//
// Created by Daniel Coburn on 9/27/24.
//
#pragma once

#include <cstdlib>
#include "Sensor.h"

template <int POLLING_RATE> struct ExampleSensor : public Sensor<POLLING_RATE> {
  struct Data {
    int a;
    int b;
  };

  Data poll() {
    return Data{
        .a = rand(),
        .b = rand(),
    };
  }
};
