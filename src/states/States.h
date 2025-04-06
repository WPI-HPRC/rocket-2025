#pragma once

#include "../Context.h"
#include "../boilerplate/StateMachine/State.h"
#include "../boilerplate/StateMachine/StateMachine.h"
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
};

class Boost : public State {
    STATE_INNER(Boost)
};

class Coast : public State {
    STATE_INNER(Coast)

    float prevAltitude = 0;
};

class DrogueDescent : public State {
    STATE_INNER(DrogueDescent)

    float prevAltitude = 0;
};

class MainDescent : public State {
    STATE_INNER(MainDescent)
};

class Recovery : public State {
    STATE_INNER(Recovery)
};

class Abort : public State {
    STATE_INNER(Abort)
};
