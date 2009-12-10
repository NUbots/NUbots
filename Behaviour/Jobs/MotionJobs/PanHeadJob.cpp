/*! @file PanHeadJob.cpp
    @brief Implementation of PanHeadJob class

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

#include "PanHeadJob.h"

/*! @brief Constructs a PanHeadJob
 
    @param period the new pan period in milliseconds
    @param centre the centre head position about which we will pan [yaw (rad), pitch (rad), roll (rad)]
    @param limits the lower and upper angle limits for the pan (rad)
 */
PanHeadJob::PanHeadJob(double period, const vector<float>& centre, const vector<float>& limits) : MotionJob(Job::MOTION_NOD)
{
    m_job_time = period;     
    m_centre_position = centre;
    m_limit_positions = limits;
}

/*! @brief WalkJob destructor
 */
PanHeadJob::~PanHeadJob()
{
    m_centre_position.clear();
    m_limit_positions.clear();
}

/*! @brief Sets the pan for the head
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
    
    @param period the new pan period in milliseconds
    @param centre the centre head position about which we will pan [yaw (rad), pitch (rad), roll (rad)]
    @param limits the lower and upper angle limits for the pan (rad)
 */
void PanHeadJob::setPan(double period, const vector<float>& centre, const vector<float>& limits)
{
    m_job_time = period;     
    m_centre_position = centre;
    m_limit_positions = limits;
}

/*! @brief Gets the pan for the head
 
    @param period the pan period in milliseconds
    @param centre the centre head position about which we will pan [yaw (rad), pitch (rad), roll (rad)]
    @param limits the lower and upper angle limits for the pan (rad)
 */
void PanHeadJob::getPan(double& period, vector<float>& centre, vector<float>& limits)
{
    period = m_job_time;
    centre = m_centre_position;
    limits = m_limit_positions;
}
