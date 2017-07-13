//
// Created by nicholas on 12/02/17.
//

#ifndef STATEMACHINE_STATE_H
#define STATEMACHINE_STATE_H


#include "StateMachine.h"
#include "Controller.h"
class StateMachine;

class State {
protected:
    Controller * control;
public:
    virtual void enter(StateMachine *, Controller *)= 0;
    virtual void exit(StateMachine *)= 0;
    virtual void update(StateMachine *)= 0;
};

//class NeutralState: public State<NeutralState> {
class NeutralState: public State {
public:
    void enter(StateMachine *, Controller *);
    void exit(StateMachine *);
    void update(StateMachine *);
};

//class LinearState: public State<LinearState> {
class LinearState: public State {
public:
    void enter(StateMachine *, Controller *);
    void exit(StateMachine *);
    void update(StateMachine *);
};

//class FaceState: public State<FaceState> {
class FaceState: public State {
public:
    void enter(StateMachine *, Controller *);
    void exit(StateMachine *);
    void update(StateMachine *);
};


#endif //STATEMACHINE_STATE_H
