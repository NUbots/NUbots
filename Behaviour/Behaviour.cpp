/*! @file Behaviour.cpp
    @brief Implementation of behaviour class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include "Behaviour.h"
#include "NUPlatform/NUSystem.h"
#include "Tools/Math/General.h"
#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;
using namespace mathGeneral;

Behaviour::Behaviour()
{
    
}


Behaviour::~Behaviour()
{
    
}

void Behaviour::process(JobList& jobs)
{

}
void Behaviour::processFieldObjects(JobList& jobs,FieldObjects* AllObjects,NUSensorsData* data, int height, int width)
{
    static int runcount = 0;
    if (runcount < 5)
    {
        vector<float> walkVector;
        walkVector.push_back(5);
        walkVector.push_back(0);
        walkVector.push_back(0);
        WalkJob* walk = new WalkJob(walkVector);
        jobs.addMotionJob(walk);
        runcount++;
    }
    else
    {
        if(nusystem->getTime() - AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() < 500)
        {
            static const float maxspeed = 10;
            float headyaw, headpitch;
            data->getJointPosition(NUSensorsData::HeadPitch,headpitch);
            data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
            float measureddistance = AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
            float balldistance;
            if (measureddistance < 46)
                balldistance = 1;
            else
                balldistance = sqrt(pow(measureddistance,2) - 46*46);
            float ballbearing = headyaw + AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
            
            vector<float> walkVector(3, 0);
            
            walkVector[0] = 10*cos(ballbearing);
            walkVector[1] = 2*sin(ballbearing);
            walkVector[2] = ballbearing/3.0;
            
            WalkJob* walk = new WalkJob(walkVector);
            jobs.addMotionJob(walk);
            TrackPoint(jobs, headyaw, headpitch, AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation(), AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing());
        }
        else
        {
            vector<float> walkVector;
            walkVector.push_back(0);
            walkVector.push_back(0);
            walkVector.push_back(0);
            WalkJob* walk = new WalkJob(walkVector);
            jobs.addMotionJob(walk);
            //debug << "WalkJob not created: STOP WALKING " << endl;
            Pan(jobs);
        }
    }
}

void Behaviour::TrackPoint(JobList& jobs, float sensoryaw, float sensorpitch, float elevation, float bearing, float centreelevation, float centrebearing)
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
    jobs.addMotionJob(head);
 
  return;
}

void Behaviour::Pan(JobList& jobs)
{
    HeadPanJob* head = new HeadPanJob(HeadPanJob::Ball);
    jobs.addMotionJob(head);
    return;
}

