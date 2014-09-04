/*! @file RoboPedestrianProvider.cpp
    @brief Implementation of RoboPedestrian behaviour class

    @author Jason Kulk, Aaron Wong
 
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

#include "RoboPedestrianProvider.h"
#include "ChaseBlueGoal.h"
#include "ChaseYellowGoal.h"
#include "SearchForBlueGoal.h"
#include "SearchForYellowGoal.h"
#include "Paused.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"



RoboPedestrianProvider::RoboPedestrianProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    m_chase_blue_goal = new ChaseBlueGoal(this);
    m_chase_yellow_goal = new ChaseYellowGoal(this);
    m_search_blue_goal = new SearchForBlueGoal(this);
    m_search_yellow_goal = new SearchForYellowGoal(this);
    m_paused = new Paused(this);
    
    m_state = m_search_blue_goal;
}

RoboPedestrianProvider::~RoboPedestrianProvider()
{
    delete m_chase_blue_goal;
    delete m_chase_yellow_goal;
    delete m_search_blue_goal;
    delete m_search_yellow_goal;
    delete m_paused;
}

BehaviourState* RoboPedestrianProvider::nextStateCommons()
{
    if (singleChestClick() or longChestClick())
    {
        if (m_state == m_paused)
            return m_previous_state;
        else
            return m_paused;
    }
    else
        return m_state;
}


