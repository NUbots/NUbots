/*! @file GoalKeeperProvider.cpp
    @brief Implementation of GoalKeeper behaviour class

    @author Aaron Wong
 
 Copyright (c) 2010 Aaron Wong
 
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

#include "GoalKeeperProvider.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/BlockJob.h"
#include "Infrastructure/Jobs/MotionJobs/ScriptJob.h"

#include "Infrastructure/FieldObjects/FieldObjects.h"

#include <math.h>
#include <numeric>
#include "debug.h"
#include "debugverbositybehaviour.h"
#include "Tools/Math/StlVector.h"



GoalKeeperProvider::GoalKeeperProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    m_block_time = boost::circular_buffer<float>(60, 60000);
}


GoalKeeperProvider::~GoalKeeperProvider()
{
}

void GoalKeeperProvider::doBehaviour()
{
    // hack it, and put the GameState into Playing
    while (m_game_info->getCurrentState() != GameInformation::PlayingState)
        m_game_info->doManualStateChange();
    
    if (Blackboard->Sensors->CurrentTime < 8000)
        Blackboard->Jobs->addMotionJob(new WalkJob(0.01,0,0));
    else if (Blackboard->Sensors->CurrentTime < 10000)
        Blackboard->Jobs->addMotionJob(new WalkJob(0,0,0));
    else
    {
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        
        // do the head motion (track ball if visible, pan if not)
        if (ball.isObjectVisible())
        {   // track the ball if it is visible
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        }
        else if (ball.TimeSinceLastSeen() > 250 and ball.TimeSinceLastSeen() < 3000)
        {   // pan for the ball if it hasn't been seen for a bit
            m_jobs->addMotionJob(new HeadPanJob(ball));
        }
        else
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
        
        std::vector<float> ball_prediction = self.CalculateClosestInterceptToMobileObject(ball);
        
        m_block_time.push_back(ball_prediction[0]);
        float time = accumulate(m_block_time.begin(), m_block_time.end(), 0.0f)/m_block_time.size();
        //debug << "Predicted intercept: " << ball_prediction << " " << ball.velX() << " " << ball.velY() << std::endl;
        
        if (ball.TimeSeen() > 2000 and time > 300 and time < 3000)
        {
            //m_jobs->addMotionJob(new BlockJob(m_data->CurrentTime + time, ball_prediction[1], ball_prediction[2]));
            m_actions->add(NUActionatorsData::Sound, m_data->CurrentTime, "error1.wav");
        }
        
        if (singleChestClick())
            m_jobs->addMotionJob(new ScriptJob(m_data->CurrentTime + 2000, "newscript"));
    }
}

