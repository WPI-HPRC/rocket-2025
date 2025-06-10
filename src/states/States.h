#pragma once

#include "../Context.h"
#include "../boilerplate/StateMachine/State.h"
#include "../boilerplate/StateMachine/StateMachine.h"
#include "../boilerplate/Utilities/Debouncer.h"
#include "FlightParams.h"
#include "boilerplate/Utilities/ComplementaryFilter.h"
#include "boilerplate/Utilities/RunningExpAverage.h"
#include "boilerplate/Utilities/TimeAverage.h"
#include <Arduino.h>

enum StateId {
    ID_PreLaunch,
    ID_Boost,
    ID_CoastWait,
    ID_CoastAirbrake,
    ID_CoastEnd,
    ID_DrogueDescent,
    ID_MainDescent,
    ID_Recovery,
    ID_Abort
};

using State = TState<Context, StateId, decltype(&millis)>;
using StateMachine = TStateMachine<Context, StateId, decltype(&millis)>;

#define STATE_CONSTRUCTOR(name)                                                \
    name(Context *ctx) : State(ID_##name, ::millis, ctx)

#define STATE_INNER                                                            \
    void initialize_impl() override;                                           \
    State *loop_impl() override;

class PreLaunch : public State {
  public:
    STATE_CONSTRUCTOR(PreLaunch) {}

  private:
    STATE_INNER

    Debouncer accelDebouncer = Debouncer(500);
    uint32_t lastAccelReadingTime = 0;
    TimeAverage<float, 50> altAverager{};
    uint32_t lastBaroReadingTime = 0;
    bool savedInitialAltitude = false;
};

class Boost : public State {
  public:
    STATE_CONSTRUCTOR(Boost) {}

  private:
    STATE_INNER

    Debouncer accelDebouncer = Debouncer(100);
    uint32_t lastAccelReadingTime = 0;
};

class CoastWait : public State {
  public:
    STATE_CONSTRUCTOR(CoastWait) {}

  private:
    STATE_INNER

    ComplementaryFilter velocityFilter{0.001};
    RunningExpAverage<float> ewma{0.3};
    float prevAltitude = 0;

    uint32_t lastBaroReadingTime = 0;
    uint32_t lastMagReadingTime = 0;
};

class CoastAirbrake : public State {
  public:
    CoastAirbrake(Context *ctx, ComplementaryFilter &velocityFilter) : State(ID_CoastAirbrake, ::millis, ctx), velocityFilter(velocityFilter) {}

  private:
    STATE_INNER

    ComplementaryFilter &velocityFilter;

    uint32_t lastBaroReadingTime = 0;
    uint32_t lastAccelReadingTime = 0;
    uint32_t lastMagReadingTime = 0;
};

class CoastEnd : public State {
  public:
    STATE_CONSTRUCTOR(CoastEnd) {}

  private:
    STATE_INNER

    // FIXME: probably way to big
    RunningExpAverage<float> ewma{0.3};
    float prevAltitude = 0;
    Debouncer velDebouncer = Debouncer(100);
    uint32_t lastBaroReadingTime = 0;
};

class DrogueDescent : public State {
  public:
    STATE_CONSTRUCTOR(DrogueDescent) {}

  private:
    STATE_INNER

    RunningExpAverage<float> ewma{0.3};
    float prevAltitude = 0;
    Debouncer velDebouncer = Debouncer(50);
    uint32_t lastBaroReadingTime = 0;
};

class MainDescent : public State {
  public:
    STATE_CONSTRUCTOR(MainDescent) {}

  private:
    STATE_INNER

    RunningExpAverage<float> ewma{0.1};
    float prevAltitude = 0;
    Debouncer velDebouncer = Debouncer(50);
    uint32_t lastBaroReadingTime = 0;
};

class Recovery : public State {
  public:
    STATE_CONSTRUCTOR(Recovery) {}

  private:
    STATE_INNER
};

class Abort : public State {
  public:
    STATE_CONSTRUCTOR(Abort) {}

  private:
    STATE_INNER
};
