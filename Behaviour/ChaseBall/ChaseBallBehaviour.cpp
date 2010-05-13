/*! @file ChaseBallBehaviour.cpp
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

#include "ChaseBallBehaviour.h"

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

ChaseBallBehaviour::ChaseBallBehaviour(AbstractBehaviour* parent)
{
    m_parent_behaviour = parent;
}


ChaseBallBehaviour::~ChaseBallBehaviour()
{
    
}

void ChaseBallBehaviour::doBehaviour()
{
    if (m_data->CurrentTime < 500)
        return;
    
    if(m_data->CurrentTime - m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() < 500)
    {
        static const float maxspeed = 10;
        float headyaw, headpitch;
        m_data->getJointPosition(NUSensorsData::HeadPitch,headpitch);
        m_data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
        float measureddistance = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
        float balldistance;
        if (measureddistance < 46)
            balldistance = 1;
        else
            balldistance = sqrt(pow(measureddistance,2) - 46*46);
        float ballbearing = headyaw + m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
        
        vector<float> walkVector(3, 0);
        
        walkVector[0] = 10*cos(ballbearing);
        walkVector[1] = 2*sin(ballbearing);
        walkVector[2] = ballbearing/3.0;
        
        WalkJob* walk = new WalkJob(walkVector);
        m_jobs->addMotionJob(walk);
        TrackPoint(headyaw, headpitch, m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation(), m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing());
    }
    else
    {
        vector<float> walkVector;
        walkVector.push_back(0);
        walkVector.push_back(0);
        walkVector.push_back(0);
        WalkJob* walk = new WalkJob(walkVector);
        m_jobs->addMotionJob(walk);
        Pan();
    }
}

void ChaseBallBehaviour::TrackPoint(float sensoryaw, float sensorpitch, float elevation, float bearing, float centreelevation, float centrebearing)
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
    m_jobs->addMotionJob(head);
 
  return;
}

void ChaseBallBehaviour::Pan()
{
    HeadPanJob* head = new HeadPanJob(HeadPanJob::Ball);
    m_jobs->addMotionJob(head);
    return;
}

