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

#include "Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "GameController/GameInformation.h"
#include "TeamInformation.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

/*! @brief Construct a behaviour provider with the given manager
 */
BehaviourFSMProvider::BehaviourFSMProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    m_state = NULL;
    m_previous_state = NULL;
    m_state_changed = false;
}

/*! @brief Destroys the behaviour provider as well as all of the associated states
 */
BehaviourFSMProvider::~BehaviourFSMProvider()
{
    for (vector<BehaviourState*>::iterator it = m_states.begin(); it != m_states.end(); ++it)
        delete (*it);
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

/*! @brief 
 */
void BehaviourFSMProvider::doBehaviour()
{
    // check for state changes
    BehaviourState* nextstate = m_state->nextState();
    if (nextstate != m_state)
    {
        m_previous_state = m_state;
        m_state = nextstate;
        m_state_changed = true;
    }
    else
        m_state_changed = false;
    
    // perform state behaviour
    m_state->doState();
}

/*! @brief Adds a state to the list of states
 */
void BehaviourFSMProvider::addState(BehaviourState* state)
{
    m_states.push_back(state);
}


