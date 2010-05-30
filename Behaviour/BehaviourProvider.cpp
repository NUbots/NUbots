/*! @file BehaviourProvider.cpp
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

#include "BehaviourProvider.h"
#include "Behaviour.h"

#include "Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include "Behaviour/Jobs/MotionJobs/MotionKillJob.h"
#include "Behaviour/Jobs/MotionJobs/MotionFreezeJob.h"
#include "Behaviour/Jobs/VisionJobs/SaveImagesJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

/*! @brief Construct a behaviour provider with the given manager
 */
BehaviourProvider::BehaviourProvider(Behaviour* manager)
{
    m_manager = manager;
    
    m_saving_images = false;
    // Initialise the private variables for button press detection
    m_chest_state = 0;
    m_chest_previous_state = 0;
    m_chest_times = boost::circular_buffer<float>(4);
    m_chest_durations = boost::circular_buffer<float>(4);
    
    m_left_state = 0;
    m_left_previous_state = 0;
    m_left_times = boost::circular_buffer<float>(4);
    m_left_durations = boost::circular_buffer<float>(4);
    
    m_right_state = 0;
    m_right_previous_state = 0;
    m_right_times = boost::circular_buffer<float>(4);
    m_right_durations = boost::circular_buffer<float>(4);
    
    m_previous_single_chest_click = 0;
    m_previous_long_chest_click = 0;
    m_previous_double_chest_click = 0;
    m_previous_triple_chest_click = 0;
    m_previous_quad_chest_click = 0;
    
    m_previous_long_left_click = 0;
    m_previous_single_left_click = 0;
    m_previous_double_left_click = 0;
    
    m_previous_long_right_click = 0;
    m_previous_single_right_click = 0;
    m_previous_double_right_click = 0;
}

BehaviourProvider::~BehaviourProvider()
{
}

/*! @brief Preprocesses the data. Returns true if the data is valid
    
    At this level in the hierarchy we perform preprocessing relevant to ALL behaviour providers.
 
    @param jobs the nubot job list
    @param data the nubot sensor data
    @param actions the nubot actionators data
    @param fieldobjects the nubot world model
    @param gameinfo the nubot game information
    @param teaminfo the nubot team information
 */
bool BehaviourProvider::preProcess(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo)
{
    if (jobs == NULL or data == NULL or actions == NULL or /*fieldobjects == NULL or*/ gameinfo == NULL or teaminfo == NULL)
        return false;
    else
    {
        m_jobs = jobs;
        m_data = data;
        m_actions = actions;
        m_field_objects = fieldobjects;
        m_game_info = gameinfo;
        m_team_info = teaminfo;
        
        m_previous_time = m_current_time;
        m_current_time = m_data->CurrentTime;
        
        updateButtonValues();
        return true;
    }
}

void BehaviourProvider::postProcess()
{
}

/*! @brief Runs the current behaviour. Note that this may not be the current behaviour.
    @param jobs the nubot job list
    @param data the nubot sensor data
    @param actions the nubot actionators data
    @param fieldobjects the nubot world model
    @param gameinfo the nubot game information
    @param teaminfo the nubot team information
 */
void BehaviourProvider::process(JobList* jobs, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects, GameInformation* gameinfo, TeamInformation* teaminfo)
{
    if (preProcess(jobs, data, actions, fieldobjects, gameinfo, teaminfo))
    {
        doBehaviour();
        postProcess();
    }
}

/*! @brief Updates the many button related variables
 */
void BehaviourProvider::updateButtonValues()
{    
    m_chest_previous_state = m_chest_state;
    m_left_previous_state = m_left_state;
    m_right_previous_state = m_right_state;
    
    vector<float> temp;
    // Get the button state values
    if (m_data->getButtonValues(NUSensorsData::MainButton, temp) && (temp.size() >= 1))
        m_chest_state = temp[0];
    
    if(m_data->getFootBumperValues(NUSensorsData::AllFeet, temp) && temp.size() >= 2)
    {
        m_left_state = temp[0];
        m_right_state = temp[1];
    }
    
    // Get the durations of the last press
    if (m_data->getButtonTriggers(temp) and temp.size() >= 3)
    {
        // update the circular buffers on negative edges
        if (m_chest_state < m_chest_previous_state)
        {
            m_chest_times.push_back(m_current_time);
            m_chest_durations.push_back(temp[0]);
        }
        if (m_left_state < m_left_previous_state)
        {
            m_left_times.push_back(m_current_time);
            m_left_durations.push_back(temp[1]);
        }
        if (m_right_state < m_right_previous_state)
        {
            m_right_times.push_back(m_current_time);
            m_right_durations.push_back(temp[2]);
        }
    }
    
    // A double chest click always starts saving images not matter which behaviour
    if (doubleChestClick())
    {
        m_saving_images = not m_saving_images;
        m_jobs->addVisionJob(new SaveImagesJob(m_saving_images, false));
    }
    
    // A triple chest click always removes the stiffness
    if (tripleChestClick())
        removeStiffness();
    
    // A quad chect click always restarts behaviour
    if (quadChestClick())
        restartBehaviour();
}

/*! @brief Removes the stiffness
 */
void BehaviourProvider::removeStiffness()
{
    m_jobs->addMotionJob(new MotionKillJob());
    m_actions->addSound(m_current_time, "remove_stiffness.wav");
}

/*! @brief Sets the behaviour provider to be the "SelectBehaviour" provider
 */
void BehaviourProvider::restartBehaviour()
{
    m_manager->setNextBehaviour("select_behaviour");
    m_jobs->addMotionJob(new MotionKillJob());
}


bool BehaviourProvider::longChestClick()
{
    return longClick(m_chest_times, m_chest_durations, m_previous_long_chest_click);
}

bool BehaviourProvider::singleChestClick()
{
    return nClick(1, m_chest_times, m_chest_durations, m_previous_single_chest_click);
}

bool BehaviourProvider::doubleChestClick()
{
    return nClick(2, m_chest_times, m_chest_durations, m_previous_double_chest_click);
}

bool BehaviourProvider::tripleChestClick()
{
    return nClick(3, m_chest_times, m_chest_durations, m_previous_triple_chest_click);
}

bool BehaviourProvider::quadChestClick()
{
    return nClick(4, m_chest_times, m_chest_durations, m_previous_quad_chest_click);
}

bool BehaviourProvider::longLeftBumperClick()
{
    return longClick(m_left_times, m_left_durations, m_previous_long_left_click);
}

bool BehaviourProvider::singleLeftBumperClick()
{
    return nClick(1, m_left_times, m_left_durations, m_previous_single_left_click);
}

bool BehaviourProvider::doubleLeftBumperClick()
{
    return nClick(2, m_left_times, m_left_durations, m_previous_double_left_click);
}

bool BehaviourProvider::longRightBumperClick()
{
    return longClick(m_right_times, m_right_durations, m_previous_long_right_click);
}

bool BehaviourProvider::singleRightBumperClick()
{
    return nClick(1, m_right_times, m_right_durations, m_previous_single_right_click);
}

bool BehaviourProvider::doubleRightBumperClick()
{
    return nClick(2, m_right_times, m_right_durations, m_previous_double_right_click);
}

/*! @brief Returns true if the last press was a long one
     @param times a circular buffer of the click times
     @param durations a circular buffer of the click durations
     @param previoustime the previous time that this event has occured
 */
bool BehaviourProvider::longClick(const boost::circular_buffer<float>& times, const boost::circular_buffer<float>& durations, float& previoustime)
{
    if (times.empty())
        return false;
    else if (previoustime == times.back())
        return false;
    else if (durations.back() <= 800)
        return false;
    else
    {
        previoustime = m_current_time;
        return true;
    }
}

/*! @brief Returns true if an n click has occured in the given times and durations
    @param n the number of 'quick' consecutive clicks
    @param times a circular buffer of the click times
    @param durations a circular buffer of the click durations (needed to throw out long clicks)
    @param previoustime the previous time that this event has occured
 */
bool BehaviourProvider::nClick(unsigned int n, const boost::circular_buffer<float>& times, const boost::circular_buffer<float>& durations, float& previoustime)
{
    size_t buffersize = times.size();
    if (buffersize < n)                              // if there aren't enough values in the buffer return false
        return false;
    else if (previoustime == times.back())           // if previous time has not changed return false
        return false;
    else if (m_current_time - times.back() < 500)    // need to wait 500 ms for a potential next click
        return false;
    else
    {
        // n click if the last n presses are each less than 400ms apart
        for (size_t i = buffersize-1; i > buffersize-n; i--)
        {
            if (times[i] - times[i-1] > 500 || durations[i] > 800)
                return false;
        }
        
        // check the n+1 click was longer than 400ms
        if (buffersize-n > 0)
        {
            if (times[buffersize-n] - times[buffersize-n-1] < 500 || durations[buffersize-n] > 800)
                return false;
        }
        
        previoustime = times.back();
        return true;
    }
}
