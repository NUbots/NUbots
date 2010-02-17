/*! @file WalkJob.cpp
    @brief Partial implementation of WalkJob class

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

#include "WalkJob.h"

WalkJob::WalkJob(const vector<float>& speed) : MotionJob(Job::MOTION_WALK)
{
    m_walk_speed = speed;
    m_job_time = 0;         // we always want the walk speed to change *now*
}

/*! @brief WalkJob destructor
 */
WalkJob::~WalkJob()
{
    m_walk_speed.clear();
}

/*! @brief Sets the speed of the walk job.
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
    
    @param newspeed the new speed for the walk job
 */
void WalkJob::setSpeed(const vector<float>& newspeed)
{
    m_walk_speed = newspeed;
}

/*! @brief Gets the speed of the walk job
    @param speed parameter that will be updated with the walk speed
 */
void WalkJob::getSpeed(vector<float>& speed)
{
    speed = m_walk_speed;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void WalkJob::summaryTo(ostream& output)
{
    output << "WalkJob: " << m_job_time << " ";
    for (unsigned int i=0; i<m_walk_speed.size(); i++)
        output << m_walk_speed[i] << ",";
    output << endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void WalkJob::csvTo(ostream& output)
{
    output << "WalkJob, " << m_job_time << ", ";
    for (unsigned int i=0; i<m_walk_speed.size(); i++)
        output << m_walk_speed[i] << ", ";
    output << endl;
}

ostream& WalkJob::operator<< (ostream& output)
{
    return output;
}

istream& WalkJob::operator>> (istream& input)
{
    return input;
}
