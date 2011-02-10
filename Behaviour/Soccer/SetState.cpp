/*! @file SetState.cpp
    @brief Implementation of the set soccer state

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

#include "SetState.h"
#include "SoccerState.h"
#include "SoccerProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"

SetState::SetState(SoccerProvider* provider) : SoccerState(provider)
{
}

SetState::~SetState()
{
}

BehaviourState* SetState::nextState()
{
    return this;
}

void SetState::doState()
{
    if (m_provider->stateChanged())
    {   // play a sound, and stop moving
        m_actions->add(NUActionatorsData::Sound, m_data->CurrentTime, NUSounds::SET);
    }
    // In set the chest led should be yellow
    vector<float> yellow(3,1);
    yellow[0] = 0.9;
    yellow[2] = 0.1;
    m_actions->add(NUActionatorsData::ChestLed, m_data->CurrentTime, yellow);
    
    // In set if we have kick off the led should be on, and off when we don't have kick off
    if (m_game_info->haveKickoff())
        m_actions->add(NUActionatorsData::RFootLed, m_data->CurrentTime, yellow);
    else
        m_actions->add(NUActionatorsData::RFootLed, m_data->CurrentTime, vector<float>(3,0));
    
    // In set we can move the head, so track the ball if you can see it otherwise do a pan
    
    if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]));
    else if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSinceLastSeen() > 250)
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
    
    // In set we must not walk
    m_jobs->addMotionJob(new WalkJob(0,0,0));
}

