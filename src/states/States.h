#pragma once

#include "../Context.h"
#include "../boilerplate/StateMachine/State.h"
#include "../boilerplate/StateMachine/StateMachine.h"
#include "../boilerplate/Utilities/Debouncer.h"
#include "FlightParams.h"
#include <Arduino.h>

enum StateId {
    ID_PreLaunch,
    ID_Boost,
    ID_Coast,
    ID_DrogueDescent,
    ID_MainDescent,
    ID_Recovery,
    ID_Abort
};

using State = TState<Context, StateId, decltype(&millis)>;
using StateMachine = TStateMachine<Context, StateId, decltype(&millis)>;

#define STATE_INNER(name)                                                      \
  public:                                                                      \
    name(Context *ctx) : State(ID_##name, ::millis, ctx) {}                    \
                                                                               \
  private:                                                                     \
    void initialize_impl() override;                                           \
    State *loop_impl() override;

class PreLaunch : public State {
    STATE_INNER(PreLaunch)

    Debouncer launchAccelDebouncer = Debouncer(500);
    uint32_t lastAccelReadingTime = 0;
};

class Boost : public State {
    STATE_INNER(Boost)
};

class Coast : public State {
    STATE_INNER(Coast)

    constexpr static float alpha = 0.5; // smoothing coefficient. 0 <= alpha <= 1. Values near 0 prioritize old values (more smoothing) and values near 1 prioritize new values (less smoothing).
    bool firstVelCalculated = false;
    float prevAltitude = 0;
    float avgBaroVel = 0;
    uint32_t lastBaroReadingTime = 0;
};

class DrogueDescent : public State {
    STATE_INNER(DrogueDescent)

    constexpr static float alpha = 0.5; // smoothing coefficient. 0 <= alpha <= 1. Values near 0 prioritize old values (more smoothing) and values near 1 prioritize new values (less smoothing).
    float prevAltitude = 0;
    bool firstVelCalculated = false;
    float avgBaroVel = 0;
    uint32_t lastBaroReadingTime = 0;
};

class MainDescent : public State {
    STATE_INNER(MainDescent)

    constexpr static float alpha = 0.2; // smoothing coefficient. 0 <= alpha <= 1. Values near 0 prioritize old values (more smoothing) and values near 1 prioritize new values (less smoothing).
    float prevAltitude = 0;
    bool firstVelCalculated = false;
    float avgBaroVel = 0;
    uint32_t lastBaroReadingTime = 0;
};

class Recovery : public State {
    STATE_INNER(Recovery)
};

class Abort : public State {
    STATE_INNER(Abort)
};
