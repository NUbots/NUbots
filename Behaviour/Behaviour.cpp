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


    if(nusystem->getTime() - AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].TimeLastSeen() < 3000)
    {
        float headYaw;
        data->getJointPosition(NUSensorsData::HeadYaw,headYaw);
        vector<float> walkVector;
        walkVector.push_back(3);
        walkVector.push_back(0);
        walkVector.push_back(headYaw/2);
        WalkJob* walk = new WalkJob(walkVector);
        jobs.addMotionJob(walk);
        //debug << "WalkJob created: Walk to BALL: "<< walkVector[0] << ","<<walkVector[1] <<"," << headYaw/2 << endl;

        
        float headPitch;
        data->getJointPosition(NUSensorsData::HeadYaw,headYaw);
        data->getJointPosition(NUSensorsData::HeadPitch,headPitch);
        TrackPoint(jobs,headYaw, headPitch, AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].ScreenX(), AllObjects->mobileFieldObjects[FieldObjects::FO_BALL].ScreenY(), height, width);
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
    HeadJob* head = new HeadJob(0,headVector);
    
    jobs.addMotionJob(head);
 
  return;
}

void Behaviour::Pan(JobList& jobs)
{
    
    vector<float> headVector;
    HeadJob* head;
    float newPan;
    float newTilt;

    newPan = sin(nusystem->getTime()/1000) * deg2rad(70.0f);
    newTilt = sin(nusystem->getTime()/200) * PI/5;
    headVector.push_back(newTilt);
    headVector.push_back(newPan);
    head = new HeadJob(10000,headVector);
    jobs.addMotionJob(head);

    //debug << "JobList Size " << jobs.size()<<  "Time: "<< nusystem->getTime()<<endl; 
 
  return;
}

