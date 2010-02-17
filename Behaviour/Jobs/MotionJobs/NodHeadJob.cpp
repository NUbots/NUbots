/*! @file NodHeadJob.cpp
    @brief Implementation of NodHeadJob class

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

#include "NodHeadJob.h"

/*! @brief Constructs a NodHeadJob
 
    @param period the new nod period in milliseconds
    @param centre the centre head position about which we will nod [yaw (rad), pitch (rad), roll (rad)]
    @param limits the lower and upper angle limits for the nod (rad)
 */
NodHeadJob::NodHeadJob(double period, const vector<float>& centre, const vector<float>& limits) : MotionJob(Job::MOTION_NOD)
{
    m_job_time = period;     
    m_centre_position = centre;
    m_limit_positions = limits;
}

/*! @brief WalkJob destructor
 */
NodHeadJob::~NodHeadJob()
{
    m_centre_position.clear();
    m_limit_positions.clear();
}

/*! @brief Sets the nod for the head
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
    
    @param period the new nod period in milliseconds
    @param centre the centre head position about which we will nod [yaw (rad), pitch (rad), roll (rad)]
    @param limits the lower and upper angle limits for the nod (rad)
 */
void NodHeadJob::setNod(double period, const vector<float>& centre, const vector<float>& limits)
{
    m_job_time = period;     
    m_centre_position = centre;
    m_limit_positions = limits;
}

/*! @brief Gets the nod for the head
 
    @param period the nod period in milliseconds
    @param centre the centre head position about which we will nod [yaw (rad), pitch (rad), roll (rad)]
    @param limits the lower and upper angle limits for the nod (rad)
 */
void NodHeadJob::getNod(double& period, vector<float>& centre, vector<float>& limits)
{
    period = m_job_time;
    centre = m_centre_position;
    limits = m_limit_positions;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void NodHeadJob::summaryTo(ostream& output)
{
    output << "NodHeadJob: " << m_job_time << " ";
    for (unsigned int i=0; i<m_centre_position.size(); i++)
        output << m_centre_position[i] << ",";
    output << " ";
    for (unsigned int i=0; i<m_limit_positions.size(); i++)
        output << m_limit_positions[i] << ",";
    output << endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void NodHeadJob::csvTo(ostream& output)
{
    output << "NodHeadJob: " << m_job_time << " ";
    for (unsigned int i=0; i<m_centre_position.size(); i++)
        output << m_centre_position[i] << ", ";
    for (unsigned int i=0; i<m_limit_positions.size(); i++)
        output << m_limit_positions[i] << ", ";
    output << endl;
}

ostream& NodHeadJob::operator<< (ostream& output)
{
    return output;
}

istream& NodHeadJob::operator>> (istream& input)
{
    return input;
}
