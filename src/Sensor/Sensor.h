//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "services/Time.h"

#include <stdlib.h>

template <class Data>
class Sensor {

private:
    Time* time;
    long lastTimeRead;
    long pollingPeriod;

protected:
    virtual Data poll() = 0;

public:

  Sensor(Time* time, long pollingPeriod):
    time(time),
    pollingPeriod(pollingPeriod),
    lastTimeRead(time->millis())
  {}
    
   virtual bool init() = 0;

   void *update() {
      long now = time->millis();
      if (now - lastTimeRead >= pollingPeriod) {
          lastTimeRead = now;
          data = poll();
          return (void *)&data;
      }

      return nullptr;
   }
   long getLastTimeRead() {
      return lastTimeRead;
   }

   virtual size_t sensorDataBytes() const = 0;
   virtual ~Sensor() = default;

   Data data;
};
