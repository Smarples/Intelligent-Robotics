//
// Created by nicholas on 12/02/17.
//

#include "State.h"
#include "StateMachine.h"
#include <chrono>

typedef std::chrono::high_resolution_clock CTime;

//Neural State Functions
void NeutralState::enter(StateMachine *sm, Controller *_control) {
    std::cout << "Entered Neutral State" <<  std::endl;
    control = _control;
    control->resetPos();
    update(sm);
}

void NeutralState::exit(StateMachine *sm) {
    control->resetPos();
}

void NeutralState::update(StateMachine *sm) {
    std::cout << "Updating Neutral State" <<  std::endl;
	auto t0 = CTime::now();
    while(true) {
        Point center = control->checkScreenFace();
        if (center.x != 0 && center.y != 0) {
            sm->changeState(new FaceState());
            return;
        }
        int conf = control->checkScreenLinear();
        if (conf > 0.5) {
            sm->changeState(new LinearState());
            return;
		}
        int circles = control->checkCircles();
        if (circles > 0)
            control->leftHandCountingToFive(circles,t0);
        else
            //control->linear_filter();
            control->resetLARM();
        control->lookAround(t0);
    }
}

//Linear State Functions
void LinearState::enter(StateMachine *sm, Controller *_control) {
	std::cout << "Entered Linear State" << std::endl;
    control = _control;
    update(sm);
}

void LinearState::exit(StateMachine *sm) {

}

void LinearState::update(StateMachine *sm) {
    std::cout << "Updating Linear State" <<  std::endl;
    auto t0 = CTime::now();
    while(true) {
        Point center = control->checkScreenFace();
        if (center.x != 0 && center.y != 0) {
            sm->changeState(new FaceState());
            return;
        }
        int conf = control->checkScreenLinear();
        if (conf > 0.5) {
            control->waveCutRight(t0);
        } else {
            control->resetRARM();
            if (conf == 0) {
                sm->changeState(new NeutralState());
                return;
            }
        }
        int circles = control->checkCircles();
        if (circles > 0)
            control->leftHandCountingToFive(circles,t0);
        else
            control->resetLARM();
    }
}

//Face State Functions
void FaceState::enter(StateMachine *sm, Controller *_control) {
	std::cout << "Entered Face State" << std::endl;
    control = _control;
    update(sm);
}

void FaceState::exit(StateMachine *sm) {
    control->resetLARM();
}

void FaceState::update(StateMachine *sm) {
    std::cout << "Updating Face State" <<  std::endl;
    auto t0 = CTime::now();
    while(true) {
        Point center = control->checkScreenFace();
        if (center.x != 0 && center.y != 0) {
            control->waveCutLeft(t0);
        } else {
            if (control->checkScreenLinear() > 0.5) {
                sm->changeState(new LinearState());
                return;
            } else {
                sm->changeState(new NeutralState());
                return;
            }
        }
        //int circles = control->checkCircles();
        //if (circles > 0)
        //    control->rightHandCountingToFive(circles,t0);
        //else
        //    control->resetRARM();
    }
}
