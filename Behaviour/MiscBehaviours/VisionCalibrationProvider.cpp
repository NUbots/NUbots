/*! @file VisionCalibrationProvider.cpp
    @brief Implementation of vision calibration behaviour class

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

#include "VisionCalibrationProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "Infrastructure/Jobs/VisionJobs/SaveImagesJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/ChaseBall/ChaseBallProvider.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "nubotdataconfig.h"

#include "debug.h"
#include "debugverbositybehaviour.h"



VisionCalibrationProvider::VisionCalibrationProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    LUTBuffer = new unsigned char[LUTTools::LUT_SIZE];
    currentLookupTable = LUTBuffer;
    loadLUTFromFile(std::string(DATA_DIR) + std::string("default.lut"));
    
    m_selection_index = 0;
    m_num_motions = 5;
    m_saving_images = false;
    m_chase_ball = NULL;
    isStart = 0;
    lastSpoken = 0;
}


VisionCalibrationProvider::~VisionCalibrationProvider()
{
    if (m_chase_ball != NULL)
        delete m_chase_ball;
}

void VisionCalibrationProvider::doBehaviour()
{
    // hack it, and put the GameState into Playing
    while (m_game_info->getCurrentState() != GameInformation::PlayingState)
        m_game_info->doManualStateChange();
    
    doSelectedMotion();
    
    // handle the selection of motions
    if (singleLeftBumperClick())
    {
        m_selection_index = (m_selection_index + 1) % m_num_motions;
        if (m_selection_index == 4)
            m_actions->add(NUActionatorsData::Sound, m_current_time, "chase_ball.wav");
        else
            m_actions->add(NUActionatorsData::Sound, m_current_time, "error1.wav");
    }
    if (singleRightBumperClick())
    {
        m_selection_index = (m_selection_index + m_num_motions-1) % m_num_motions;
        if (m_selection_index == 4)
            m_actions->add(NUActionatorsData::Sound, m_current_time, "chase_ball.wav");
        else
            m_actions->add(NUActionatorsData::Sound, m_current_time, "error1.wav");
    }
    
    // handle the starting and stopping of saving images
    if (singleChestClick())
    {
        m_saving_images = not m_saving_images;
        m_jobs->addVisionJob(new SaveImagesJob(m_saving_images, true));
    }
}

void VisionCalibrationProvider::doSelectedMotion()
{
    if (m_selection_index == 0)
    {   // start with no stiffness
        std::vector<float> zero(m_actions->getSize(NUActionatorsData::Head), 0);
        m_actions->add(NUActionatorsData::Head, m_current_time, zero, 0);
        m_jobs->addMotionJob(new WalkJob(0,0,0));
        if(m_current_time - lastSpoken > 10000)
        {
            lastSpoken = m_current_time;
            //CLASSIFY WHOLE IMAGE:
            
            int unclassifiedCounter = classifyImage();
            //CALCULATE PERCENTAGE CLASSIFIED:
            //DONE IN VISION
            float percentage = ((Blackboard->Image->getTotalPixels() - unclassifiedCounter) / (Blackboard->Image->getTotalPixels()*1.00) ) * 100.00;
            //SAY PERCENTAGE:
            sayPercentageClassified(percentage);
        }
        
        if (isStart < 50)
        {
            m_jobs->addMotionJob(new WalkJob(0.001,0.001,0.001));
            isStart++;
        }
        else
        {
            m_jobs->addMotionJob(new WalkJob(0,0,0));
        }
    }
    else if (m_selection_index == 1)
    {   // generic pan
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
        m_jobs->addMotionJob(new WalkJob(0,0,0));
    }
    else if (m_selection_index == 2)
    {   
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Ball));
        m_jobs->addMotionJob(new WalkJob(0,0,0));
    }
    else if (m_selection_index == 3)
    {
        std::vector<float> zero(m_actions->getSize(NUActionatorsData::Head), 0);
        m_actions->add(NUActionatorsData::Head, m_current_time, zero, 50.0);
    }
    else if (m_selection_index == 4)
    {
        if (m_chase_ball == NULL)
            m_chase_ball = new ChaseBallProvider(m_manager, false);
        m_chase_ball->process(m_jobs, m_data, m_actions, m_field_objects, m_game_info, m_team_info);
    }
}
 
void VisionCalibrationProvider::sayPercentageClassified(float percentage)
{
	int tens = int(floor(fabs(percentage) /10.0));
	int units = int(fabs(percentage) - tens*10);
	
	std::string unit_numbers[] = {"0","1","2","3","4","5","6","7","8","9","10"};
	std::string teen_numbers[] = {"10","11","12","13","14","15","16","17","18","19"};
	std::string ten_numbers[] ={"0","10","20","30","40","50","60","70","80","90","100","110","120"};
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
	std::vector<std::string> sounds (2);
	sounds[0] = ten_numbers[tens] + ".wav";
	sounds[1] = unit_numbers[units] + ".wav";
	m_actions->add(NUActionatorsData::Sound, m_current_time, sounds);
	
	return;
}
                             
void VisionCalibrationProvider::loadLUTFromFile(const std::string& fileName)
{
    LUTTools lutLoader;
    if (lutLoader.LoadLUT(LUTBuffer, LUTTools::LUT_SIZE,fileName.c_str()) == true)
        currentLookupTable = LUTBuffer;
    else
        errorlog << "Behaviour::VisionCalibration::loadLUTFromFile(" << fileName << "). Failed to load lut." << std::endl;
}

int VisionCalibrationProvider::classifyImage()
{
    int unclassifiedCounter = 0;
    int width = Blackboard->Image->getWidth();
    int height = Blackboard->Image->getHeight();

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            NUImage* image = Blackboard->Image;
            Pixel temp = (*image)(x,y);
            unsigned char classifiedPixel = currentLookupTable[LUTTools::getLUTIndex(temp)];
            if(classifiedPixel  == Vision::unclassified || temp.y < 50)
            {
                unclassifiedCounter++;
            }
        }
    }
    return unclassifiedCounter;
}
    


