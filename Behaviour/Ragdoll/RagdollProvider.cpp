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

#include "RagdollProvider.h"
#include "RagdollStates.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"
#include <iostream>
#include "debugverbositybehaviour.h"



RagdollProvider::RagdollProvider(Behaviour* manager, bool pauseable) : BehaviourFSMProvider(manager)
{
    m_paused_state = new RagdollState(this);
    
    m_state = m_paused_state;
}


RagdollProvider::~RagdollProvider()
{
    delete m_paused_state;
}

BehaviourState* RagdollProvider::nextStateCommons()
{
    return m_state;
}


