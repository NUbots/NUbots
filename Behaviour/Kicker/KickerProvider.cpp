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

#include "KickerProvider.h"
#include "KickerStates.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Behaviour/Jobs/MotionJobs/HeadJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

KickerProvider::KickerProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    m_wait_state = new WaitState(this);
    m_kick_state = new KickState(this);
    m_state = m_wait_state;
}


KickerProvider::~KickerProvider()
{
    delete m_wait_state;
    delete m_kick_state;
}

BehaviourState* KickerProvider::nextStateCommons()
{
    // we can pause the chase ball in any state
    if ((singleChestClick() || longChestClick()))
    {
        if (m_state == m_wait_state)
            return m_kick_state;
//        else if (m_state == m_kick_state)
//            return m_wait_state;
        else
            return m_state;
    }
    else
        return m_state;
}


