/*! @file ChaseBallProvider.cpp
    @brief Implementation of chase ball behaviour class

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

#include "ChaseBallProvider.h"
#include "ChaseBallStates.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "GameController/GameInformation.h"
#include "Behaviour/TeamInformation.h"

#include "Behaviour/Jobs/MotionJobs/HeadJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

ChaseBallProvider::ChaseBallProvider(Behaviour* manager, bool pauseable) : BehaviourFSMProvider(manager)
{
    m_pauseable = pauseable;
    m_chase_state = new ChaseState(this);
    addState(m_chase_state);
    m_search_state = new SearchState(this);
    addState(m_search_state);
    m_position_state = new PositionState(this);
    addState(m_position_state);
    m_paused_state = new PausedState(this);
    addState(m_paused_state);
    
    if (m_pauseable)
        m_state = m_paused_state;
    else
        m_state = m_search_state;
}


ChaseBallProvider::~ChaseBallProvider()
{
}

BehaviourState* ChaseBallProvider::nextStateCommons()
{
    // we can pause the chase ball in any state
    if (m_pauseable and (singleChestClick() or longChestClick()))
    {
        if (m_state == m_paused_state)
            return m_search_state;
        else
            return m_paused_state;
    }
    else
        return m_state;
}


