/*! @file SoccerProvider.cpp
    @brief Implementation of soccer behaviour class

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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

#include "SoccerProvider.h"

#include "IntialState.h"
#include "ReadyState.h"
#include "SetState.h"
#include "PlayingState.h"
#include "PenalisedState.h"
#include "SubstituteState.h"
#include "RequiresSubstituteState.h"

#include "Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

/*! @brief Construct a behaviour provider with the given manager
 */
SoccerProvider::SoccerProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    m_initial = new InitialState(this);
    m_ready = new ReadyState(this);
    m_set = new SetState(this);
    m_playing = new PlayingState(this);
    m_penalised = new PenalisedState(this);
    m_substitute = new SubstituteState(this);
    m_requires_substitution = new RequiresSubstituteState(this);
    
    m_state = m_initial;
}

/*! @brief Destroys the behaviour provider as well as all of the associated states
 */
SoccerProvider::~SoccerProvider()
{
    delete m_initial;
    delete m_ready;
    delete m_set;
    delete m_playing;
    delete m_penalised;
    delete m_substitute;
    delete m_requires_substitution;
}

/*! @brief Performs behaviour that is common to all states in the soccer behaviour provider
 */
void SoccerProvider::doBehaviourCommons()
{
}

/*! @brief Checks for state transitions that are common to all states in this behaviour provider
 */
BehaviourState* SoccerProvider::nextStateCommons()
{
    // if button is pressed we go into penalised or playing
    return m_state;
}


