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

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

VisionCalibrationProvider::VisionCalibrationProvider(Behaviour* manager) : BehaviourProvider(manager)
{
    m_selection_index = 0;
    m_num_motions = 5;
    m_saving_images = false;
    m_chase_ball = NULL;
}


VisionCalibrationProvider::~VisionCalibrationProvider()
{
    if (m_chase_ball != NULL)
        delete m_chase_ball;
}

void VisionCalibrationProvider::doBehaviour()
{
    doSelectedMotion();
    
    // handle the selection of motions
    if (singleLeftBumperClick())
    {
        m_selection_index = (m_selection_index + 1) % m_num_motions;
        if (m_selection_index == 4)
            m_actions->addSound(m_current_time, "chase_ball.wav");
        else
            m_actions->addSound(m_current_time, "error1.wav");
    }
    if (singleRightBumperClick())
    {
        m_selection_index = (m_selection_index + m_num_motions-1) % m_num_motions;
        if (m_selection_index == 4)
            m_actions->addSound(m_current_time, "chase_ball.wav");
        else
            m_actions->addSound(m_current_time, "error1.wav");
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
    {
        vector<float> zero(m_actions->getNumberOfJoints(NUActionatorsData::HeadJoints), 0);
        m_actions->addJointPositions(NUActionatorsData::HeadJoints, m_current_time, zero, zero, 50);
        m_jobs->addMotionJob(new WalkJob(0,0,0));
        //! @todo TODO: If we are not standing up, then we should stand up (probably need a stand-up job, or set the walk speed non-zero for a little bit)
    }
    else if (m_selection_index == 1)
    {
        vector<float> zero(m_actions->getNumberOfJoints(NUActionatorsData::HeadJoints), 0);
        m_actions->addJointPositions(NUActionatorsData::HeadJoints, m_current_time, zero, zero, -100);
        m_jobs->addMotionJob(new WalkJob(0,0,0));
    }
    else if (m_selection_index == 2)
    {
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Ball));
        m_jobs->addMotionJob(new WalkJob(0,0,0));
    }
    else if (m_selection_index == 3)
    {
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
        m_jobs->addMotionJob(new WalkJob(0,0,0));
    }
    else if (m_selection_index == 4)
    {
        if (m_chase_ball == NULL)
            m_chase_ball = new ChaseBallProvider(m_manager, false);
        m_chase_ball->process(m_jobs, m_data, m_actions, m_field_objects, m_game_info, m_team_info);
    }
}
 

                             
    


