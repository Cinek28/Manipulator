/**
 * @file StateMachine.h
 * @author Marcin Baran (marcinb280@op.pl)
 * @brief State machine pattern class which is responsible for processing
 *        (invoking and changing) states based on delivered transition table
 *        (defined as map with key as state names and value as map of pointers
 *        to states (value) and triggers (key) directing how to change to given 
 *        states). To work correctly, before running, it should have defined 
 *        transition table, starting state and set of end states names to know
 *        when to stop processing.
 * @version 0.1
 * @date 2019-05-30
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include "State.h"

/**
 * @brief Transition table used to decide what state should be processed next.
 *        Based on the state the state machine is currently in and trigger returned
 *        after processing it, the lookup to transition table decides which state
 *        should be processed next. Example:
 *        TransitionTable table;
 *        table["StartState"] = {
 *                                  {StateTrigger::Trigger::TO_CONTINUE_TRIGGER, std::make_shared<StopState>()},
 *                              };
 *        table["StopState"] = {
 *                                  {StateTrigger::Trigger::TO_END_TRIGGER, std::make_shared<EndState>()},
 *                             };
 *        table["StopState"] = {}; //Empty state suggesting it is ending state
 *  
 * 
 */
typedef std::unordered_map<std::string, std::unordered_map<StateTrigger::Trigger, std::shared_ptr<State>>> TransitionTable;

/**
 * @brief State machine pattern class which is responsible for processing
 *        (invoking and changing) states based on delivered transition table
 *        (defined as map with key as state names and value as map of pointers
 *        to states (value) and triggers (key) directing how to change to given 
 *        states). To work correctly, before running, it should have defined 
 *        transition table, starting state and set of end states names to know
 *        when to stop processing.
 * 
 */
class StateMachine
{
public:
    /**
     * @brief Construct a new State Machine object based on given TransitionTable
     * 
     * @param transitionTable : based on this table state machine decides which state should 
     *                          be invoked next when in current state and according to trigger
     *                          current state returned
     */
    explicit StateMachine(TransitionTable& transitionTable):transitionTable(transitionTable){};

    /**
     * @brief Initializes shared_ptr to starting state of state machine and set of end states.
     *        Must be called before running state machine.
     * 
     * @param state 
     */
    void initState(State& state, const std::unordered_set<std::string>& endStates);

    /**
     * @brief Runs state machine continuously based on given TranistionTable. Stops when
     *        one of end states is reached or next state is null.
     * 
     */
    void run();

    /**
     * @brief Runs processing of current state, then changes the state to next invoking 
     *        onExit() for current state and then onEntry() for the next.
     * 
     * @return true : when the next state is one of end states or null
     * @return false : when the next state is valid non-ending state
     */
    bool runOnce();

private:
    std::shared_ptr<State> currentState = nullptr;
    std::unordered_set<std::string> endStates = {};
    TransitionTable transitionTable = {};

    /**
     * @brief If possible (next state is not null) changes to next state invoking 
     *        onExit() for current state and onEntry() for the next. If next state
     *        is ending state returns true, otherwise false. The next state is chosen
     *        based on TransitionTable.
     * 
     * @param trigger : a trigger which causes state to change
     * 
     * @return true : when the next state is one of end states or null
     * @return false : when the next state is valid non-ending state
     */
    bool changeState(const StateTrigger::Trigger& trigger);
};
