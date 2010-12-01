/*! @file VisionCalibrationProvider.cpp
    @brief Implementation of Pose behaviour class

    @author Aaron Wong
 
 Copyright (c) 2010 Aaron Wong
 
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

#include "PoseProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"


#include <math.h>
#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

PoseProvider::PoseProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    m_pitch_index = 0;
    m_yaw_index = 25;
    m_num_pitch_motions = 15;
    m_num_yaw_motions = 49;
    isStart = 0;
    m_saving_images = false;

}


PoseProvider::~PoseProvider()
{

}

void PoseProvider::doBehaviour()
{
    	
    if(singleChestClick())
    {
	m_saving_images = true;
        m_jobs->addVisionJob(new SaveImagesJob(m_saving_images, true));
	

    }
    else if(m_saving_images == true)
    {
	m_saving_images = false;
        m_jobs->addVisionJob(new SaveImagesJob(m_saving_images, true));
    }
    
    // handle the selection of motions
    if (singleLeftBumperClick())
    {
        m_pitch_index = (m_pitch_index + 1) % m_num_pitch_motions;
	//m_actions->addSound(m_current_time, "error1.wav");
	sayPosition(calculatePitchPosition());
    }
    if (singleRightBumperClick())
    {
        m_yaw_index = (m_yaw_index + 1) % m_num_yaw_motions;
	//m_actions->addSound(m_current_time, "error1.wav");
	sayPosition(calculateYawPosition());
    }
    if(doubleLeftBumperClick())
    {
	m_pitch_index = (m_pitch_index  + m_num_pitch_motions-1) % m_num_pitch_motions;
	//m_actions->addSound(m_current_time, "error1.wav");
	sayPosition(calculatePitchPosition());
    }
    if(doubleRightBumperClick())
    {
	m_yaw_index = (m_yaw_index  + m_num_yaw_motions-1) % m_num_yaw_motions;
	//m_actions->addSound(m_current_time, "error1.wav");
	sayPosition(calculateYawPosition());
    }
    doSelectedMotion();
    
}

void PoseProvider::doSelectedMotion()
{
    //Initialisation: First 50 Frames, will be used to stand up
    if (isStart < 50)
    {
        vector<float> zero(m_actions->getSize(NUActionatorsData::Head), 0);
        m_actions->add(NUActionatorsData::Head, m_current_time, zero, 50);
        m_jobs->addMotionJob(new WalkJob(0.001,0.001,0.001));
		isStart++;
    }
    //Start the bahaviour:
    else
    {
        //vector<float> zero(m_actions->getNumberOfJoints(NUActionatorsData::HeadJoints), 0);
        vector<float> position(3);
        //POSITION [PITCH, YAW, ROLL]
        position[0] = calculatePitchPosition()*3.14/180;
        position[1] = calculateYawPosition()*3.14/180; //degrees to radians
        position[2] = 0;
        m_jobs->addMotionJob(new HeadJob(m_current_time,position));
        m_jobs->addMotionJob(new WalkJob(0,0,0));
    }
}
 
float PoseProvider::calculatePitchPosition()
{
	float min_pitch = -40.0; //-38.5
	float max_pitch = 30; //29.5
	float degrees = ((m_pitch_index)*(max_pitch - min_pitch)/(m_num_pitch_motions-1)) + min_pitch;

	return degrees;
	return degrees;
}
    
float PoseProvider::calculateYawPosition()
{
	float min_yaw = -120.0; //-119.5
	float max_yaw = 120.0; //119.5
	float degrees = ((m_yaw_index)*(max_yaw - min_yaw)/(m_num_yaw_motions-1)) + min_yaw;
	return degrees;
}

void PoseProvider::sayPosition(float degrees)
{
	int tens = int(floor(fabs(degrees) /10.0));
	int units = int(fabs(degrees) - tens*10);
	
	string unit_numbers[] = {"0","1","2","3","4","5","6","7","8","9","10"};
	string teen_numbers[] = {"10","11","12","13","14","15","16","17","18","19"};
	string ten_numbers[] ={"0","10","20","30","40","50","60","70","80","90","100","110","120"};
	if(tens == 1)
	{
		m_actions->add(NUActionatorsData::Sound, m_current_time, teen_numbers[units]  + ".wav");
		return;
	}
	if(tens == 0)
	{
		m_actions->add(NUActionatorsData::Sound, m_current_time, unit_numbers[units] + ".wav");
		return;
	}
	if(units == 0 && tens !=0)
	{
		m_actions->add(NUActionatorsData::Sound, m_current_time, ten_numbers[tens] + ".wav");
		return;
	}
	if(units == 0 && tens == 0)
	{
		m_actions->add(NUActionatorsData::Sound, m_current_time, unit_numbers[units] + ".wav");
		return;
	}
	vector<string> sounds (2);
	sounds[0] = ten_numbers[tens] + ".wav";
	sounds[1] = unit_numbers[units] + ".wav";
	m_actions->add(NUActionatorsData::Sound, m_current_time, sounds);
	
	return;
}

