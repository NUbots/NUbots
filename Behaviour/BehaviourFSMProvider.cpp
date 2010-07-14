/*! @file BehaviourFSMProvider.cpp
    @brief Implementation of behaviour class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "BehaviourFSMProvider.h"
#include "Behaviour.h"
#include "BehaviourState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

/*! @brief Construct a behaviour provider with the given manager
 */
BehaviourFSMProvider::BehaviourFSMProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    m_state = NULL;
    m_previous_state = NULL;
    m_state_changed = true;
}

/*! @brief Destroys the behaviour provider as well as all of the associated states
 */
BehaviourFSMProvider::~BehaviourFSMProvider()
{
}

/*! @brief Runs the current behaviour. Note that this may not be the current behaviour.
    @param jobs the nubot job list
    @param data the nubot sensor data
    @param actions the nubot actionators data
    @param fieldobjects the nubot world model
    @param gameinfo the nubot game information
    @param teaminfo the nubot team information
 */
void BehaviourFSMProvider::process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo)
{
    if (preProcess(jobs, data, actions, fieldobjects, gameinfo, teaminfo))
    {
        doBehaviour();
        postProcess();
    }
}

/*! @brief Returns true if the state has changed */
bool BehaviourFSMProvider::stateChanged()
{
    return m_state_changed;
}

/*! @brief Returns true if state was in fact the previous state, and false if it was not */
bool BehaviourFSMProvider::wasPreviousState(BehaviourState* state)
{
    return (state == m_previous_state);
}

/*! @brief Returns a pointer to the previous state */
BehaviourState* BehaviourFSMProvider::getPreviousState()
{
    return m_previous_state;
}

/*! @brief 
 */
void BehaviourFSMProvider::doBehaviour()
{
    // do behaviour common to all states
    doBehaviourCommons();
    
    // check for state changes common to all states
    BehaviourState* nextstate = nextStateCommons();
    if (nextstate == m_state)       // then check for state change specific to this state
        nextstate = m_state->getNextState();
    
    // do state transition
    if (nextstate != m_state)
    {
        m_previous_state = m_state;
        m_state = nextstate;
        m_state_changed = true;
    }
    else
        m_state_changed = false;
    
    // perform state behaviour
    m_state->process(m_jobs, m_data, m_actions, m_field_objects, m_game_info, m_team_info);
}

/*! @brief Performs behaviour that is common to all states in this behaviour provider
 */
void BehaviourFSMProvider::doBehaviourCommons()
{
    return;
}

/*! @brief Checks for state transitions that are common to all states in this behaviour provider
 */
BehaviourState* BehaviourFSMProvider::nextStateCommons()
{
    return m_state;
}


