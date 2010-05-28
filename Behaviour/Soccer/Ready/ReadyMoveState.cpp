/*! @file ReadyMoveState.cpp
    @brief Implementation of the initial soccer state

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

#include "ReadyMoveState.h"

#include "Behaviour/Jobs/JobList.h"
#include "Vision/FieldObjects/FieldObjects.h"

#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"

ReadyMoveState::ReadyMoveState(SoccerFSMState* parent) : SoccerState(parent)
{
}

ReadyMoveState::~ReadyMoveState()
{
}

BehaviourState* ReadyMoveState::nextState()
{
    return this;
}

void ReadyMoveState::doState()
{
    if (m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST]));
    else if (m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST]));
    else if (m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST]));
    else if (m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST].isObjectVisible())
        m_jobs->addMotionJob(new HeadTrackJob(m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST]));
    else
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
    
    vector<float> result = m_field_objects->self.CalculateDifferenceFromFieldState(vector<float>(3,0));
    float speed = 0.5*result[0];
    float x = (speed/5)*cos(result[1]);
    float y = speed*sin(result[1]);
    float yaw = result[2]/4.0;
    
    m_jobs->addMotionJob(new WalkJob(x,y,yaw));
}

