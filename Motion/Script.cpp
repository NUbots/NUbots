/*! @file Script.cpp
    @brief Implementation of Script class

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

#include "Script.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/Jobs/MotionJobs/ScriptJob.h"
#include "NUWalk.h"

#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for Script module
 */
Script::Script(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUMotionProvider("Script", data, actions)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "Script::Script()" << std::endl;
#endif
    m_walk = walk;
    m_script_start_time = -1;
    m_script_pending = false;
}

/*! @brief Destructor for FallProtection module
 */
Script::~Script()
{
}

void Script::stop()
{
}

void Script::stopHead()
{
}

void Script::stopArms()
{
}

void Script::stopLegs()
{
}

void Script::kill()
{
    if (isActive())
    {   // if the script is currently running, the only way to kill it is to set the stiffnesses to 0
        m_script_start_time = 0;
        
        std::vector<float> sensor_larm, sensor_rarm;
        std::vector<float> sensor_lleg, sensor_rleg;
        m_data->getPosition(NUSensorsData::LArm, sensor_larm);
        m_data->getPosition(NUSensorsData::RArm, sensor_rarm);
        m_data->getPosition(NUSensorsData::LLeg, sensor_lleg);
        m_data->getPosition(NUSensorsData::RLeg, sensor_rleg);
        
        m_actions->add(NUActionatorsData::LLeg, 0, sensor_lleg, 0);
        m_actions->add(NUActionatorsData::RLeg, 0, sensor_rleg, 0);
        m_actions->add(NUActionatorsData::LArm, 0, sensor_larm, 0);
        m_actions->add(NUActionatorsData::RArm, 0, sensor_rarm, 0);
    }
}

/*! @brief Returns true is a script is currently being executed */
bool Script::isActive()
{
    return m_data->CurrentTime <= m_script.timeFinished() + m_script_start_time;
}

bool Script::isUsingHead()
{
    return m_data->CurrentTime <= m_script.timeFinishedWithHead() + m_script_start_time;
}

bool Script::isUsingArms()
{
    return m_data->CurrentTime <= std::max(m_script.timeFinishedWithLArm(),m_script.timeFinishedWithRArm()) + m_script_start_time;
}

/*! @brief Returns true if a script uses the legs */
bool Script::isUsingLegs()
{
    return m_data->CurrentTime <= std::max(m_script.timeFinishedWithLLeg(),m_script.timeFinishedWithRLeg()) + m_script_start_time;
}

bool Script::isReady()
{
    return m_script.isValid();
}

bool Script::requiresHead()
{
    return m_script.usesHead();
}

bool Script::requiresArms()
{
    return m_script.usesLArm() or m_script.usesRArm();
}

bool Script::requiresLegs()
{
    return m_script.usesLLeg() or m_script.usesRLeg();
}

/*! @brief Produce actions from the data to move the robot into a standing position
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions to move into the protection pose.
 */
void Script::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (data == NULL || actions == NULL)
        return;
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "Script::process()" << std::endl;
    #endif
    if (m_script_pending and m_script_start_time < m_data->CurrentTime)
    {
        m_script.play(data, actions);
        m_script_start_time = m_data->CurrentTime;
        m_script_pending = false;
    }
}

/*! @brief Processes a script job
 */
void Script::process(ScriptJob* job)
{
    #if DEBUG_NUMOTION_VERBOSITY > 2
        debug << "Script::process() ";
        job->summaryTo(debug);
    #endif
    job->getScript(m_script_start_time, m_script);
    if (not m_script.isValid())
        m_script_start_time = -1;
    else
        m_script_pending = true;
}


