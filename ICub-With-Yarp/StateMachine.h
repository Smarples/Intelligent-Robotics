#ifndef STATEMACHINE_STATEMACHINE_H
#define STATEMACHINE_STATEMACHINE_H


#include "State.h"
#include "Controller.h"

class State;

class StateMachine {
    Controller * control;
    State * state;
public:
    StateMachine(Controller *);
    void changeState(State *);
    State *getState();
};


#endif //STATEMACHINE_STATEMACHINE_H
