//
// Created by Daniel Coburn on 9/27/24.
//
#pragma once

#include <stddef.h>
#include <stdint.h>

#define FIELD(type, name, period) type##_##name name;
#define UPDATE(type, name, period)                                             \
  if (now - name._lastTimeRead >= period) {                                    \
    typename type::Data data = type::poll();                                   \
                                                                               \
    name._lastTimeRead += period;                                              \
                                                                               \
    *(typename type::Data *)buf = data;                                        \
                                                                               \
    size_t dataSize = sizeof(typename type::Data);                             \
                                                                               \
    n += dataSize;                                                             \
    buf += dataSize;                                                           \
    *includedSensors |= 1 << index;                                            \
  }                                                                            \
  index++;
#define SIZE(type, name, period) n += sizeof(typename type::Data);
#define STRUCT(type, name, period)                                             \
  struct type##_##name {                                                       \
    long _lastTimeRead;                                                        \
    typename type::Data data;                                                  \
                                                                               \
    typename type::Data poll() {                                               \
      data = type::poll();                                                     \
      return data;                                                             \
    }                                                                          \
  };

#define CREATE_SENSORS(sensors)                                                \
  sensors(STRUCT);                                                             \
  struct Sensors {                                                             \
    sensors(FIELD);                                                            \
                                                                               \
    constexpr size_t largestBufferSize() {                                     \
      size_t n = 0;                                                            \
      sensors(SIZE);                                                           \
      return n;                                                                \
    }                                                                          \
                                                                               \
    size_t update(uint8_t *includedSensors, long now, uint8_t *buf,            \
                  size_t maxLen) {                                             \
      size_t n = 0;                                                            \
      size_t index = 0;                                                        \
      sensors(UPDATE);                                                         \
                                                                               \
      return n;                                                                \
    }                                                                          \
  };
