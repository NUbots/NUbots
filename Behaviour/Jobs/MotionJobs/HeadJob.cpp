/*! @file HeadJob.cpp
    @brief Implementation of HeadJob class

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

#include "HeadJob.h"

HeadJob::HeadJob(double time, const vector<float>& position) : MotionJob(Job::MOTION_HEAD)
{
    m_job_time = time;     
    m_head_position = position;
}

/*! @brief WalkJob destructor
 */
HeadJob::~HeadJob()
{
    m_head_position.clear();
}

/*! @brief Sets the position for the head to point
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
    
    @param newposition the new position for the head job [x(cm), y(cm), theta(rad)]
 */
void HeadJob::setPosition(double time, const vector<float>& newposition)
{
    m_job_time = time;
    m_head_position = newposition;
}

/*! @brief Gets the position of the head job
    @param position parameter that will be updated with the point we want to head
 */
void HeadJob::getPosition(double& time, vector<float>& position)
{
    time = m_job_time;
    position = m_head_position;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void HeadJob::summaryTo(ostream& output)
{
    output << "HeadJob: " << m_job_time << " ";
    for (unsigned int i=0; i<m_head_position.size(); i++)
        output << m_head_position[i] << ",";
    output << endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void HeadJob::csvTo(ostream& output)
{
    output << "HeadJob, " << m_job_time << ", ";
    for (unsigned int i=0; i<m_head_position.size(); i++)
        output << m_head_position[i] << ", ";
    output << endl;
}

ostream& HeadJob::operator<< (ostream& output)
{
    return output;
}

istream& HeadJob::operator>> (istream& input)
{
    return input;
}
