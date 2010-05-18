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

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"

class ChaseBallState : public BehaviourState
{
public:
    ChaseBallState(ChaseBallProvider* provider) {m_provider = provider;};
protected:
    ChaseBallProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class PausedState : public ChaseBallState
{
public:
    PausedState(ChaseBallProvider* provider) : ChaseBallState(provider) {};
    BehaviourState* nextState() {return m_provider->m_state;};
    void doState() 
    {
        if (m_provider->m_state_changed)
        {
            m_provider->m_jobs->addMotionJob(new WalkJob(0,0,0));
            vector<float> zero(m_provider->m_actions->getNumberOfJoints(NUActionatorsData::HeadJoints), 0);
            m_provider->m_jobs->addMotionJob(new HeadJob(m_provider->m_current_time + 500, zero));
        }
    };
};

// ----------------------------------------------------------------------------------------------------------------------- ChaseState
class ChaseState : public ChaseBallState
{
public:
    ChaseState(ChaseBallProvider* provider) : ChaseBallState(provider) {};
    BehaviourState* nextState()
    {
        if (m_provider->m_current_time - m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() > 500)
            return m_provider->m_search_state;
        else
            return m_provider->m_state;
    };
    
    void doState()
    {
        float headyaw, headpitch;
        m_provider->m_data->getJointPosition(NUSensorsData::HeadPitch,headpitch);
        m_provider->m_data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
        float measureddistance = m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
        float balldistance;
        if (measureddistance < 46)
            balldistance = 1;
        else
            balldistance = sqrt(pow(measureddistance,2) - 46*46);
        float ballbearing = headyaw + m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
        
        vector<float> walkVector(3, 0);
        
        walkVector[0] = 10*cos(ballbearing);
        walkVector[1] = 2*sin(ballbearing);
        walkVector[2] = ballbearing/2.0;
        
        WalkJob* walk = new WalkJob(walkVector);
        m_provider->m_jobs->addMotionJob(walk);
        TrackPoint(headyaw, headpitch, m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation(), m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing());
    };
    
    void TrackPoint(float sensoryaw, float sensorpitch, float elevation, float bearing, float centreelevation = 0, float centrebearing = 0)
    {
        const float gain_pitch = 0.8;           // proportional gain in the pitch direction
        const float gain_yaw = 0.6;             // proportional gain in the yaw direction
        
        float c_pitch = -centreelevation;
        float c_yaw = -centrebearing;
        
        float e_pitch = c_pitch + elevation;    // the sign convention of the field objects is the opposite of the head pitch joint itself
        float e_yaw = c_yaw - bearing;
        
        float new_pitch = sensorpitch - gain_pitch*e_pitch;
        float new_yaw = sensoryaw - gain_yaw*e_yaw;
        
        static vector<float> headtarget(2,0);
        headtarget[0] = new_pitch;
        headtarget[1] = new_yaw;
        HeadJob* head = new HeadJob(0, headtarget);
        m_provider->m_jobs->addMotionJob(head);
    };
};

// ----------------------------------------------------------------------------------------------------------------------- SearchState
class SearchState : public ChaseBallState
{
public:
    SearchState(ChaseBallProvider* provider) : ChaseBallState(provider) {};
    BehaviourState* nextState()
    {
        if (m_provider->m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSeen() > 0)
            return m_provider->m_chase_state;
        else
            return m_provider->m_state;
    };
    void doState()
    {
        m_provider->m_jobs->addMotionJob(new WalkJob(0,0,0.3));
        m_provider->m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::Ball, 0.3));
    };
};


#endif

