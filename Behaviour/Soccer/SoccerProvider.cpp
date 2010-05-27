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

#include "InitialState.h"
#include "ReadyState.h"
#include "SetState.h"
/*#include "PlayingState.h"*/
#include "FinishedState.h"
#include "PenalisedState.h"
/*#include "SubstituteState.h"
#include "RequiresSubstituteState.h"*/

#include "Behaviour/Jobs/JobList.h"
#include "Behaviour/GameInformation.h"
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
    /*m_playing = new PlayingState(this);*/
    m_finished = new FinishedState(this);
    m_penalised = new PenalisedState(this);
    /*m_substitute = new SubstituteState(this);
    m_requires_substitution = new RequiresSubstituteState(this);*/
    
    m_state = m_initial;
}

/*! @brief Destroys the behaviour provider as well as all of the associated states
 */
SoccerProvider::~SoccerProvider()
{
    delete m_initial;
    delete m_ready;
    delete m_set;
    //delete m_playing;
    delete m_finished;
    delete m_penalised;
    /*delete m_substitute;
    delete m_requires_substitution;*/
}

/*! @brief Performs behaviour that is common to all states in the soccer behaviour provider
 */
void SoccerProvider::doBehaviourCommons()
{
    // In every state the left foot led must display the team colour
    if (m_game_info->getTeamColour() == GameInformation::BlueTeam)
        m_actions->addLeds(NUActionatorsData::LeftFootLeds, m_current_time, 0, 0, 1);
    else
        m_actions->addLeds(NUActionatorsData::LeftFootLeds, m_current_time, 1, 0, 1);
}

/*! @brief Checks for state transitions that are common to all states in this behaviour provider
 */
BehaviourState* SoccerProvider::nextStateCommons()
{
    if (singleChestClick() or longChestClick())
        m_game_info->doManualStateChange();
    
    GameInformation::RobotState game_state = m_game_info->getCurrentState();
    switch (game_state) 
    {
        case GameInformation::InitialState:
            return m_initial;
            break;
        case GameInformation::ReadyState:
            return m_ready;
            break;
        case GameInformation::SetState:
            return m_set;
            break;
        /*case GameInformation::PlayingState:
            return m_playing;
            break;*/
        case GameInformation::FinishedState:
            return m_finished;
            break;
        case GameInformation::PenalisedState:
            return m_penalised;
            break;/*
        case GameInformation::SubstituteState:
            return m_substitute;
            break;
        case GameInformation::RequiresSubstitutionState:
            return m_requires_substitution;
            break;*/
        default:
            break;
    }
    return m_state;
}


