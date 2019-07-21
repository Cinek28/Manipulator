#include <stdexcept>
#include <iostream>
#include "StateMachine.h"

void StateMachine::initState(State& start, const std::unordered_set<std::string>& end)
{
    currentState.reset(&start);
    endStates = end;
}

bool StateMachine::changeState(const StateTrigger::Trigger& state)
{
    if(!currentState || endStates.find(currentState->name()) != endStates.end())
    {
        std::cout << "Reached end state or current state is NULL" << std::endl;
        return true;
    }
    
    State* tempState;
    try
    {
        tempState = transitionTable[currentState->name()].at(state).get();
    }catch(const std::out_of_range& error)
    {
        std::cout << error.what() << "\nNo such transition from:" << *currentState << std::endl;
        return true;
    }
    
    if(tempState->name() != currentState->name())
    {
        currentState->onExit();
        std::cout << "Changing state from " << *currentState << " to " << *tempState << std::endl;
        currentState = transitionTable[currentState->name()].at(state);
        currentState->onEntry();
    }
    return false;
}

void StateMachine::run()
{
    if(!currentState)
    {
        std::cout << "Starting state not initialized" << std::endl;
        return;
    }

    while(!runOnce());
}

bool StateMachine::runOnce()
{
    return changeState(currentState->process());   
}
