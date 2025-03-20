#pragma once

#include "boilerplate/Sensors/Impl/ASM330.h"
#include "boilerplate/Sensors/Impl/ICM20948.h"
#include "boilerplate/Sensors/Impl/LPS22.h"

struct Context {
  ASM330* accel;
  Barometer* baro;
  ICM20948* icm;
};
