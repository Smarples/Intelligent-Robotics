#include "StateMachine.h"
#include "State.h"

void StateMachine::changeState(State * newState) {
    state->exit(this);
    state = newState;
    newState->enter(this, control);
}

State *StateMachine::getState() {
    return state;
}

StateMachine::StateMachine(Controller * _control) {
    std::cout << "Created State Machine" <<  std::endl;
    control = _control;

    control->optionsHead.put("device", "remote_controlboard");
    control->optionsHead.put("local", "/tutorial/motor/head");
    control->optionsHead.put("remote", "/icubSim/head");
    control->optionsRARM.put("device", "remote_controlboard");
    control->optionsRARM.put("local", "/tutorial/motor/RARM");
    control->optionsRARM.put("remote", "/icubSim/right_arm");
    control->optionsLARM.put("device", "remote_controlboard");
    control->optionsLARM.put("local", "/tutorial/motor/LARM");
    control->optionsLARM.put("remote", "/icubSim/left_arm");
    
    PolyDriver robotHead(control->optionsHead);
    PolyDriver robotRARM(control->optionsRARM);
    PolyDriver robotLARM(control->optionsLARM);
    
    robotHead.view(control->posHead);
    robotHead.view(control->velHead);
    robotHead.view(control->encHead);
    robotRARM.view(control->posRARM);
    robotRARM.view(control->velRARM);
    robotRARM.view(control->encRARM);
    robotLARM.view(control->posLARM);
    robotLARM.view(control->velLARM);
    robotLARM.view(control->encLARM);

    state = new NeutralState();
    state->enter(this, control);
}
