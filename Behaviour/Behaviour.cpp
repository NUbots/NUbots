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
            float headyaw;
            data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
            float measureddistance = AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
            float balldistance;
            if (measureddistance < 46)
                balldistance = 1;
            else
                balldistance = sqrt(pow(measureddistance,2) - 46*46);
            float ballbearing = headyaw + AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
            
            vector<float> walkVector(3, 0);
            if (balldistance > 30)
                walkVector[0] = maxspeed;
            else
                walkVector[0] = maxspeed*(balldistance/30);
            if (fabs(ballbearing) > 0.05)
                walkVector[2] = 2*walkVector[0]*sin(ballbearing)/balldistance;
            //WalkJob* walk = new WalkJob(walkVector);
            //jobs.addMotionJob(walk);
            //debug << "WalkJob created: Walk to BALL: "<< walkVector[0] << ","<<walkVector[1] <<"," << headYaw/2 << endl;
            
            float headpitch;
            data->getJointPosition(NUSensorsData::HeadPitch,headpitch);
            TrackPoint(jobs, headyaw, headpitch, AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].ScreenX(), AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].ScreenY(), height, width);
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

void Behaviour::TrackPoint(JobList& jobs,float currPan, float currTilt, float x, float y, int IMAGE_HEIGHT, int IMAGE_WIDTH)
{

    double FOVx = deg2rad(45.0f);
    double FOVy = deg2rad(34.45f);
    float radsPerPixelHoriz = FOVx/IMAGE_WIDTH;
    float radsPerPixelVert = FOVy/IMAGE_HEIGHT;

    float cx = (IMAGE_WIDTH/2); // Center X -> desired position
    float cy = (IMAGE_HEIGHT/2); // Center Y -> desired position

    float xError = cx - x;
    float yError = cy - y;

    float alphaX = 0.8; //0.33;
    float alphaY = 0.8; //0.5;

    float angErrX = alphaX*xError*radsPerPixelHoriz;
    float angErrY = alphaY*yError*radsPerPixelVert;

    float newPan = currPan + angErrX;
    float newTilt = currTilt - angErrY;
    //float newTilt = currTilt;

    //const float CAMERA_BOTTOM_MIN_TILT = -0.2f;

    vector<float> headVector;
    headVector.push_back(newTilt);
    headVector.push_back(newPan);
    HeadJob * head = new HeadJob(0,headVector);
    
    jobs.addMotionJob(head);
 
  return;
}

void Behaviour::Pan(JobList& jobs)
{
    HeadPanJob* head = new HeadPanJob(HeadPanJob::Ball);
    jobs.addMotionJob(head);
    return;
}

