/*! @file CameraCalibrationProvider.cpp
    @brief Implementation of CameraCalibrationProvider class

    @author David Budden
 
 Copyright (c) 2010 David Budden
 
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

#include "CameraCalibrationProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"
#include "Infrastructure/Jobs/CameraJobs/ChangeCameraSettingsJob.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUImage/NUImage.h"

#include <math.h>
#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

CameraCalibrationProvider::CameraCalibrationProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    m_pitch_index = 0;
    m_yaw_index = 0;
    isStart = 0;
    m_saving_images = false;
    topcam = false;
    angleOffset = 9.0;
}


CameraCalibrationProvider::~CameraCalibrationProvider()
{

}

void CameraCalibrationProvider::doBehaviour()
{   
    
    // handle the selection of motions
    CameraSettings settings;
	
    if (singleLeftBumperClick())
    {
       if (!topcam)
       // change camera and position
       {
           m_pitch_index = (17.4 - angleOffset)*3.14/180;
           m_actions->add(NUActionatorsData::Sound, m_current_time, "error1.wav");
           topcam = true;
           //settings.activeCamera = CameraSettings::TOP_CAMERA;
           //m_camera->setSettings(settings);
       }
       else
       // copy image
       {             
             refImage.copyFromExisting(*(Blackboard->Image));
             m_actions->add(NUActionatorsData::Sound, m_current_time, "camera_click.wav");
             
             //Pixel tmp = refImage.m_image[1][1];
             //debug << "Test BB image Read: \tY1: " << (int)tmp.yCbCrPadding << ",\t\tU: " << (int)tmp.cb << ",\t\tY2: "<< (int)tmp.y << ",\t\tV: " << (int)tmp.cr << endl;             
             
             Pixel tmp;
             for (int i = 0; i < 320; i++)             
             {
                 for (int j = 0; j < 240; j++)
                 {
                     tmp = refImage.m_image[j][i];
                     debug << "Pixel Data: \tY1: " << (int)tmp.yCbCrPadding << "\t\tU: " << (int)tmp.cb << "\t\tY2: "<< (int)tmp.y << "\t\tV: " << (int)tmp.cr << "\t\t(" << i << "," << j << ")" << endl;       
                 }
             }
       }

	   //settings.activeCamera = 0;
    }
    
    if (singleRightBumperClick())
    {
       if (topcam)
       // change camera and position
       {
           m_pitch_index = -27.4*3.14/180;
           m_actions->add(NUActionatorsData::Sound, m_current_time, "error1.wav");
           topcam = false;
           
           //settings.activeCamera = CameraSettings::BOTTOM_CAMERA;
           //m_camera->setSettings(settings);
       }
       else
       // copy image
       {
             refImage.copyFromExisting(*(Blackboard->Image));
             m_actions->add(NUActionatorsData::Sound, m_current_time, "camera_click.wav");  
             
             Pixel tmp;
             for (int i = 0; i < 320; i++)             
             {
                 for (int j = 0; j < 240; j++)
                 {
                     tmp = refImage.m_image[j][i];
                     debug << "Pixel Data: \tY1: " << (int)tmp.yCbCrPadding << "\t\tU: " << (int)tmp.cb << "\t\tY2: "<< (int)tmp.y << "\t\tV: " << (int)tmp.cr << "\t\t(" << i << "," << j << ")" << endl;       
                 }
             }
       }
	   
	   //settings.activeCamera = 1;
    }
	
	
	//m_jobs->addCameraJob(new ChangeCameraSettingsJob(settings));
    

    doSelectedMotion();
    
}

void CameraCalibrationProvider::doSelectedMotion()
{
    //Initialisation: First 50 Frames, will be used to stand up
    if (isStart < 50)
    {
    // stand up
        vector<float> zero(m_actions->getSize(NUActionatorsData::Head), 0);
        m_actions->add(NUActionatorsData::Head, m_current_time, zero, 50);
        m_jobs->addMotionJob(new WalkJob(0.001,0.001,0.001));
		isStart++;
    }
    else if (isStart < 51)
    // set initial head position and camera
    {
         m_pitch_index = -27.4*3.14/180;
         //m_actions->add(NUActionatorsData::Sound, m_current_time, "error1.wav");
         topcam = false;  
         isStart++;        
    }
    //Start the bahaviour:
    else
    {        
        vector<float> position(3);
        //POSITION [PITCH, YAW, ROLL]
        
        position[0] = m_pitch_index;
        position[1] = 0;
        position[2] = 0;
        m_jobs->addMotionJob(new HeadJob(m_current_time,position)); //
        m_jobs->addMotionJob(new WalkJob(0,0,0));                   // stop
    }
    
    //debug << "Test BB image Read: " << Blackboard->Image->getHeight() << ", "<< Blackboard->Image->getWidth() <<endl;
    //image[y][x]
    
    //Pixel tmp = Blackboard->Image->m_image[1][1];
    //Pixel tmp = refImage.m_image[1][1];
    
    //debug << "Test BB image Read: \tY1: " << (int)tmp.yCbCrPadding << ",\t\tU: " << (int)tmp.cb << ",\t\tY2: "<< (int)tmp.y << ",\t\tV: " << (int)tmp.cr << endl;
    
    
}

