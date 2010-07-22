/*! @file ChaseBallStates.h
    @brief Chase ball states

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

#ifndef CHASEBALLSTATES_H
#define CHASEBALLSTATES_H

#include "Behaviour/BehaviourState.h"
#include "ChaseBallProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"

#include "debug.h"

class ChaseBallSubState : public BehaviourState
{
public:
    ChaseBallSubState(ChaseBallProvider* provider){m_provider = provider;};
protected:
    ChaseBallProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class PausedState : public ChaseBallSubState
{
public:
    PausedState(ChaseBallProvider* provider) : ChaseBallSubState(provider) {};
    BehaviourState* nextState() {return m_provider->m_state;};
    void doState() 
    {
        if (m_provider->m_state_changed)
            m_provider->m_jobs->addMotionJob(new MotionFreezeJob());
    };
};

// ----------------------------------------------------------------------------------------------------------------------- ChaseState
class ChaseBallState : public ChaseBallSubState
{
public:
    ChaseBallState(ChaseBallProvider* provider) : ChaseBallSubState(provider) {};
    BehaviourState* nextState()
    {
        if (m_provider->m_current_time - m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() > 500)
        {
            debug << "Chase -> Search" << endl;
            return m_provider->m_search_state;
        }
        else if (not m_provider->m_team_info->amIClosestToBall())
        {
            debug << "Chase -> Position" << endl;
            return m_provider->m_position_state;
        }
        else
            return m_provider->m_state;
    };
    
    void doState()
    {
        if (m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
        {
            float headyaw, headpitch;
            m_provider->m_data->getJointPosition(NUSensorsData::HeadPitch,headpitch);
            m_provider->m_data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
            float measureddistance = m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
            float balldistance = measureddistance * cos(m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
            float ballbearing = m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
            
            float trans_speed = 1;
            float trans_direction = ballbearing;
            float yaw = ballbearing/2;
            
            if (true or balldistance > 15)
            {
                vector<float> temp;
                float leftobstacle = 255;
                float rightobstacle = 255;
                if (m_data->getDistanceLeftValues(temp))
                    leftobstacle = temp[0];
                if (m_data->getDistanceRightValues(temp))
                    rightobstacle = temp[0];
                
                if (leftobstacle < 50)
                {
                    trans_speed = trans_speed + 0.01*(leftobstacle - 50);
                    if (trans_speed > 0)
                        trans_direction = trans_direction - atan2(20, leftobstacle);
                    else
                        trans_direction = trans_direction + atan2(20, leftobstacle);

                    yaw = yaw - 0.015*(leftobstacle - 50);
                }
                else if (rightobstacle < 50)
                {
                    trans_speed = trans_speed + 0.01*(rightobstacle - 50);
                    if (trans_speed > 0)
                        trans_direction = trans_direction + atan2(20, leftobstacle);
                    else
                        trans_direction = trans_direction - atan2(20, leftobstacle);
                    yaw = yaw + 0.015*(rightobstacle - 50);
                }
            }
            WalkJob* walk = new WalkJob(trans_speed, trans_direction, yaw);
            m_provider->m_jobs->addMotionJob(walk);
            HeadTrackJob* head = new HeadTrackJob(m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]);
            m_provider->m_jobs->addMotionJob(head);
        }
    };
};

// ----------------------------------------------------------------------------------------------------------------------- PositonState
class PositionState : public ChaseBallState
{
public:
    PositionState(ChaseBallProvider* provider) : ChaseBallState(provider) {};
    BehaviourState* nextState()
    {
        if (m_provider->m_current_time - m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() > 500)
        {
            debug << "Position -> Search" << endl;
            return m_provider->m_search_state;
        }
        else if (m_provider->m_team_info->amIClosestToBall())
        {
            debug << "Position -> Chase" << endl;
            return m_provider->m_chase_state;
        }
        else
            return m_provider->m_state;
    };
    
    void doState()
    {
        if (m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isObjectVisible())
        {
            float headyaw, headpitch;
            m_provider->m_data->getJointPosition(NUSensorsData::HeadPitch,headpitch);
            m_provider->m_data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
            
            float measureddistance = m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
            float balldistance = measureddistance * cos(m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
            //float balldistance;
            //if (measureddistance < 46)
            //    balldistance = 1;
            //else
            //    balldistance = sqrt(pow(measureddistance,2) - 46*46);
            float ballbearing = m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
            
            vector<float> walkVector(3, 0);
            walkVector[1] = 2*sin(ballbearing);
            walkVector[2] = ballbearing/2.0;
            walkVector[0] = 0.5*(balldistance - 100)*cos(ballbearing);
            
            vector<float> temp;
            float leftobstacle = 255;
            float rightobstacle = 255;
            
            if (m_data->getDistanceLeftValues(temp))
                leftobstacle = temp[0];
            if (m_data->getDistanceRightValues(temp))
                rightobstacle = temp[0];
            
            if (leftobstacle < 50)
            {
                walkVector[0] = walkVector[0] + 0.5*(leftobstacle - 50);
                walkVector[1] = -2;
                walkVector[2] = walkVector[2] + 0.015*(leftobstacle - 50);
            }
            else if (rightobstacle < 50)
            {
                walkVector[0] = walkVector[0] + 0.5*(rightobstacle - 50);
                walkVector[1] = 2;
                walkVector[2] = walkVector[2] - 0.015*(rightobstacle - 50);
            }
            
            WalkJob* walk = new WalkJob(0.5*(balldistance - 100), ballbearing, ballbearing/2.0);
            m_provider->m_jobs->addMotionJob(walk);
            
            HeadTrackJob* head = new HeadTrackJob(m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL]);
            m_provider->m_jobs->addMotionJob(head);
        }
    };
};

// ----------------------------------------------------------------------------------------------------------------------- SearchState
class SearchState : public ChaseBallSubState
{
public:
    SearchState(ChaseBallProvider* provider) : ChaseBallSubState(provider) {};
    BehaviourState* nextState()
    {
        if (m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSeen() > 0)
        {
            debug << "Search -> Chase" << endl;
            return m_provider->m_chase_state;
        }
        else
            return m_provider->m_state;
    };
    void doState()
    {
        float spin = 0;
        if (m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing() < 0)
            spin = -0.4;
        else
            spin = 0.4;
        m_provider->m_jobs->addMotionJob(new WalkJob(0,0,spin));
        m_provider->m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::Ball,spin));
    };
};


#endif

