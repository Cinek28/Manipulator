/**
 * @file State.h
 * @author Marcin Baran (marcinb280@op.pl)
 * @brief Base abstract class for constructing state used in state machine
 *        implementation. All created states should derive from State class
 *        and implement at least process() function. Enum class trigger is 
 *        used for determining when one state should be changed on another.
 *        It is decided by returning user-defined trigger value from process
 *        function. Functions onEntry() and onExit() are optional and are 
 *        invoked when there is a change to this state (onEntry()) or leaving
 *        the state (onExit()).
 * @version 0.1
 * @date 2019-05-30
 * 
 * @copyright Copyright (c) 2019
 * 
 */
//#include <iostream>
//#include <sstream>

/**
 * @brief Trigger class should be defined as names of triggers that causes 
 *        state to change. Example definition:
 *        enum class StateTrigger::Trigger
 *        {
 *          TO_CONTINUE_TRIGGER,
 *          TO_END_TRIGGER
 *        }
 */

namespace StateTrigger
{
    enum class Trigger;
};

/**
 * @brief Abstract class representing one of state machine's state.
 *        All created states should derive from State class and implement at
 *        least process() function. Enum class trigger is used for determining 
 *        when one state should be changed on another. It is decided by returning 
 *        user-defined trigger value from process function. 
 *        Functions onEntry() and onExit() are optional and are invoked when there 
 *        is a change to this state (onEntry()) or leaving the state (onExit()).
 * 
 */
struct State
{
    /**
     * @brief Construct a new State object
     * 
     * @param name : name of the state (obligatory, used in transition table) 
     */
    explicit State(std::string name): stateName(name){};

    /**
     * @brief Destroy the State object
     * 
     */
    virtual ~State(){};

    /**
     * @brief Optional user-defined function which is invoked when the state
     *        is changed from different kind of state.
     * 
     */
    virtual void onEntry(){};

    /**
     * @brief Optional user-defined function which is invoked when the state
     *        is changed to different kind of state.
     * 
     */
    virtual void onExit(){};

    /**
     * @brief Main obligatory state function. It is invoked each time there is 
     *        transition to this state (even if previous state was the same).
     *        Example:
     *        class StartState: public State
     *        {
     *          StartState():State("StartState"){}
     *          virtual StateTrigger::Trigger process()
     *          {
     *              std::cout << "Processing " << stateName << std::endl;
     *              return  StateTrigger::Trigger::STOP;
     *          }
     *        };
     * 
     * @return StateTrigger::Trigger : Function returns trigger which informs which
     *         state should be next based on transitionTable in StateMachine.
     *
     */
    virtual StateTrigger::Trigger process() = 0;

    /**
     * @brief Returns state name
     * 
     * @return const std::string& : name of the state
     */
    const std::string& name() const {return stateName;};

    /**
     * @brief Operator << used for string streams operation.
     * 
     * @param os : string stream to which state name should be transfered
     * @param state : name of the state
     * @return std::ostream& : reference returning modified stream
     */
    friend inline std::ostream &operator<<(std::ostream &os, const State &state)
    {
        return os << state.stateName;
    };

protected:

    std::string stateName = "Undefined";
};
