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
#include "Tools/Optimisation/PSOOptimiser.h"
#include "Tools/Optimisation/EHCLSOptimiser.h"

#include <math.h>
#include "debug.h"
#include "debugverbositybehaviour.h"
#include "nubotdataconfig.h"


#define debug_out debug_file
#define evaluations 100
#define repeatCount 10



CameraCalibrationProvider::CameraCalibrationProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    m_pitch_index = 0;
    m_yaw_index = 0;
    isStart = 0;
    m_saving_images = false;
    topcam = false;
    angleOffset = 9.0;      //10.0.1.21
    optCount = 0;
    
    // load initial camera settings from file
    
    CameraSettings settings = CameraSettings(CONFIG_DIR + std::string("Camera.cfg"));                        // loading from file needs to be changed
    //m_settings = CameraSettings(CONFIG_DIR + std::string("Camera.cfg"));
    
    m_optimiser = new PSOOptimiser("CameraSettings", settings.getAsParameters());                     // new CameraSettings.getAsParameters() required
    //m_optimiser = new EHCLSOptimiser("CameraSettings", settings.getAsParameters());
    
           
    std::stringstream debugLogName;
    debugLogName << "/var/volatile/" ;
    //if(playerNumber) debugLogName << playerNumber;
    debugLogName << "optimisation.log";
    debug_file.open(debugLogName.str().c_str());
    debug_file.clear();
    debug_file << "OPTIMISATION LOG\n" << std::endl;
    
    debug_file << "\nINITIALISE OPTIMISER\n" << std::endl;
    //debug_file << settings.getAsParameters() << std::endl;
    debug_file << m_settings.getAsParameters() << std::endl;
    debug_file << std::endl; 
}


CameraCalibrationProvider::~CameraCalibrationProvider()
{
    debug_file.close();
}

void CameraCalibrationProvider::doBehaviour()
{   
    static bool process_image = false;
    static bool ready = false;
    // handle the selection of motions
    CameraSettings settings;
	
    if (singleLeftBumperClick())
    {
       m_pitch_index = (17.4 - angleOffset)*3.14/180;
       m_actions->add(NUActionatorsData::Sound, m_current_time, "error1.wav");
       topcam = true;       
       process_image = false;    
    }
    
    if (singleRightBumperClick())
    {
       m_pitch_index = -27.4*3.14/180;
       m_actions->add(NUActionatorsData::Sound, m_current_time, "error1.wav");
       topcam = false;           
       process_image = false;  
    }	
    
    if (singleChestClick())
    {
       ready = true;
       m_actions->add(NUActionatorsData::Sound, m_current_time, "camera_click.wav"); 
    }
    if (optCount == evaluations - 1)
       m_actions->add(NUActionatorsData::Sound, m_current_time, "camera_click.wav");
       
    if (ready and isStart > 50 and optCount < evaluations)           
       process_image = true;
    else
       process_image = false;              
        

    if (process_image)
       imageProcess(); 

    doSelectedMotion();
    
}

void CameraCalibrationProvider::doSelectedMotion()
{
    //Initialisation: First 50 Frames, will be used to stand up
    if (isStart < 50)
    {
    // stand up
        std::vector<float> zero(m_actions->getSize(NUActionatorsData::Head), 0);
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
        std::vector<float> position(3);
        //POSITION [PITCH, YAW, ROLL]
        
        position[0] = m_pitch_index;
        position[1] = 0;
        position[2] = 0;
        m_jobs->addMotionJob(new HeadJob(m_current_time,position)); //
        m_jobs->addMotionJob(new WalkJob(0,0,0));                   // stop
    }    
    
}

int CameraCalibrationProvider::evaluate(std::vector< std::vector<int> > coordinates)
{
    int min = 1000;
    
    for (int i = 0; i < 8; i++)
    {
        for (int j = i+1; j < 8; j++)
        {
            //debug << "Distance: " << distance(coordinates[i], coordinates[j]) << std::endl;
         // IGNORE GREY AND SHADOW BLUE
            if (i != 3 and j != 3 and i != 7 and j != 7)
            {
                if (distance(coordinates[i], coordinates[j]) < min)
                {                                         
                   min = distance(coordinates[i], coordinates[j]);                   
                }   
            }                    
        }       
    }   
    
    return min;
}

int CameraCalibrationProvider::distance(std::vector<int> p, std::vector<int> q)
{
    return sqrt((q[0]-p[0])*(q[0]-p[0]) + (q[1]-p[1])*(q[1]-p[1]) + (q[2]-p[2])*(q[2]-p[2]));
}

void CameraCalibrationProvider::imageProcess()
{
     std::vector< std::vector<int> > coordinates;
     static float efficiency = 0;
     static int i = 0;
     
     refImage.copyFromExisting(*(Blackboard->Image));
     
     
     //debug << "\nWHITE" << std::endl;
     coordinates.push_back(getColourAvg(80,60));     
     
     //debug << "\nYELLOW" << std::endl;
     coordinates.push_back(getColourAvg(160,60));    
     
     //debug << "\nORANGE" << std::endl;
     coordinates.push_back(getColourAvg(240,60));                     
     
     //debug << "\nGREY std::endl" << std::endl;
     coordinates.push_back(getColourAvg(80,120));                    
     
     //debug << "\nGREEN" << std::endl;
     coordinates.push_back(getColourAvg(160,120));                     
     
     //debug << "\nPINK" << std::endl;
     coordinates.push_back(getColourAvg(240,120));                     
     
     //debug << "\nBLUE" << std::endl;
     coordinates.push_back(getColourAvg(80,180));                     
     
     //debug << "\nSHADOW BLUE" << std::endl;
     coordinates.push_back(getColourAvg(160,180));
     
     efficiency += (float)evaluate(coordinates)/256.0;
     //debug << "\nEfficiency:\t" << efficiency << "\n" << std::endl;    
     
     i++;
     
     if (i == repeatCount)
     {   
         //debug << i << std::endl;  // delete me      
         debug_file << optCount << ", " << (efficiency/repeatCount*100) << ", " << m_parameters << std::endl;
         m_optimiser->setParametersResult(efficiency/repeatCount*25);  // blame Jason     
         
         m_parameters = m_optimiser->getNextParameters();   
         CameraSettings settings = CameraSettings(m_parameters);                              // new constructor required
         
         //Blackboard->Jobs->addCameraJob(new ChangeCameraSettingsJob(settings));         
         //Blackboard->Jobs->addCameraJob(new ChangeCameraSettingsJob(m_settings));
         
         efficiency = 0;
         i = 0;
         optCount++;       
        
     }     
}

std::vector<int> CameraCalibrationProvider::getColourAvg(const int x, const int y)
{
     Pixel tmp;
     
     int y1_temp = 0;
     int y2_temp = 0;
     int cb_temp = 0;
     int cr_temp = 0;             
                 
     for (int i = x-10; i < x+10; i++)             
     {
         for (int j = y-10; j < y+10; j++)
         {
             tmp = refImage(i,j);
             y1_temp += (int)tmp.yCbCrPadding;
             y2_temp += (int)tmp.y;
             cb_temp += (int)tmp.cb;
             cr_temp += (int)tmp.cr;
             
             //debug << "Pixel Data: \tY1: " << (int)tmp.yCbCrPadding << "\t\tU: " << (int)tmp.cb << "\t\tY2: "<< (int)tmp.y << "\t\tV: " << (int)tmp.cr << "\t\t(" << i << "," << j << ")\n" << std::endl;       
         }
     }
     
     y1_temp /= 400;
     y2_temp /= 400;
     cb_temp /= 400;
     cr_temp /= 400;     
     
     std::vector<int> YUV, RGB;
     YUV.push_back((int)y2_temp);
     YUV.push_back((int)cb_temp);
     YUV.push_back((int)cr_temp);
     
     RGB = YUV2RGB(YUV);
     
     //debug << "\nR: " << RGB[0] << std::endl; 
     //debug << "G: " << RGB[1] << std::endl; 
     //debug << "B: " << RGB[2] << std::endl << std::endl;
     
     return RGB;
}

std::vector<int> CameraCalibrationProvider::YUV2RGB(std::vector<int> YUV)
{
     std::vector<int> RGB;     
     int c, d, e;
     
     c = YUV[0] - 16;
     d = YUV[1] - 128;
     e = YUV[2] - 128;
     
     RGB.push_back(clip((298*c + 409*e + 128) >> 8));
     RGB.push_back(clip((298*c - 100*d - 208*e + 128) >> 8));
     RGB.push_back(clip((298*c + 516*d + 128) >> 8));     
     
     return RGB;          
}

int CameraCalibrationProvider::clip(int input)
{
    if (input > 255) {return 255;}
    else if (input < 0) {return 0;}
    else return input;
}

