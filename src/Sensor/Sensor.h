//
// Created by Daniel Coburn on 9/27/24.
//
#pragma once

#include <stddef.h>
#include <stdint.h>

template <long POLLING_PERIOD> struct Sensor {
  long _lastTimeRead = 0;

  constexpr long getPollingPeriod() { return POLLING_PERIOD; }
};

template <class... Ss> struct Sensors;

template <class S, class... Ss> struct Sensors<S, Ss...> {
  Sensors<Ss...> rest;

  S mySensor;
  inline size_t update(uint8_t *includedSensors, long now, uint8_t *buf, size_t maxLen, size_t index = 0) {
    size_t n = 0;
    if (now - mySensor._lastTimeRead >= mySensor.getPollingPeriod()) {
      typename S::Data data = mySensor.poll();

      mySensor._lastTimeRead += mySensor.getPollingPeriod();

      *(typename S::Data *)buf = data;

      size_t dataSize = sizeof(typename S::Data);

      n = dataSize;
      *includedSensors |= 1 << index;
    }

    return n + rest.update(includedSensors, now, buf + n, maxLen - n, index + 1);
  }

  constexpr size_t largestBufferSize() const {
    return sizeof(typename S::Data) + rest.largestBufferSize();
  }
};

template <> struct Sensors<> {
  constexpr size_t update(uint8_t *includedSensors, long now, uint8_t *buf, size_t maxLen, size_t index) { return 0; }

  constexpr size_t largestBufferSize() const { return 0; }
};
