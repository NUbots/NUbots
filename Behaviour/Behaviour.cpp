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
            float headYaw;
            data->getJointPosition(NUSensorsData::HeadYaw,headYaw);
            vector<float> walkVector;
            walkVector.push_back(0);
            walkVector.push_back(0);
            walkVector.push_back(0);
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
    static double lastpantime = 0;
    if (nusystem->getTime() - lastpantime > 3200)
    {
        lastpantime = nusystem->getTime();
        vector<double> times(8, 0);
        vector<vector<float> > positions(8, vector<float> (2,0));
        times[0] = nusystem->getTime() + 100;
        positions[0][0] = 0.43;
        positions[0][1] = 0.81;
        
        times[1] = nusystem->getTime() + 500;
        positions[1][0] = 0.43;
        positions[1][1] = -0.81;
        
        times[2] = nusystem->getTime() + 700;
        positions[2][0] = 0.06;
        positions[2][1] = -0.81;
        
        times[3] = nusystem->getTime() + 1200;
        positions[3][0] = 0.06;
        positions[3][1] = 0.81;
        
        times[4] = nusystem->getTime() + 1400;
        positions[4][0] = -0.27;
        positions[4][1] = 0.81;
        
        times[5] = nusystem->getTime() + 1600;
        positions[5][0] = -0.50;
        positions[5][1] = 1.3;
        
        times[6] = nusystem->getTime() + 3000;
        positions[6][0] = -0.50;
        positions[6][1] = -1.3;
        
        times[7] = nusystem->getTime() + 3200;
        positions[7][0] = 0;
        positions[7][1] = 0;
        
        HeadJob* head = new HeadJob(times, positions);

        jobs.addMotionJob(head);
    }
 
  return;
}

