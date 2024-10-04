//
// Created by Daniel Coburn on 9/27/24.
//
#pragma once

#include <cstdlib>

namespace ExampleSensor {
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
} // namespace ExampleSensor
