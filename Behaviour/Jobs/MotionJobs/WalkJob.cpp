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
#include "debug.h"

/*! @brief Constructs a WalkJob

    @param speed the speed for the walk job [x (cm/s), y (cm/s), theta (rad/s)]
 */
WalkJob::WalkJob(const vector<float>& speed) : MotionJob(Job::MOTION_WALK)
{
    m_walk_speed = speed;
    m_job_time = 0;         // we always want the walk speed to change *now*
}

/*! @brief Constructs a WalkJob from stream data
    @param speed the speed for the walk job [x (cm/s), y (cm/s), theta (rad/s)]
 */
WalkJob::WalkJob(istream& input) : MotionJob(Job::MOTION_WALK)
{
    m_job_time = 0;
    char buffer[1024];
    // read in the head_position size
    input.read(buffer, sizeof(unsigned int));
    unsigned int m_walk_speed_size = *reinterpret_cast<unsigned int*>(buffer);
    
    // read in the head_position vector
    m_walk_speed = vector<float>(m_walk_speed_size, 0);
    for (unsigned int i=0; i<m_walk_speed_size; i++)
    {
        input.read(buffer, sizeof(float));
        m_walk_speed[i] = *reinterpret_cast<float*>(buffer);
    }
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

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void WalkJob::toStream(ostream& output) const
{
    debug << "WalkJob::toStream" << endl;
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
                                            // Then we write WalkJob specific data
    unsigned int m_walk_speed_size = m_walk_speed.size();
    output.write((char*) &m_walk_speed_size, sizeof(m_walk_speed_size));
    for (unsigned int i=0; i<m_walk_speed_size; i++)
        output.write((char*) &m_walk_speed[i], sizeof(m_walk_speed[i]));
}

/*! @relates WalkJob
    @brief Stream insertion operator for a WalkJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const WalkJob& job)
{
    debug << "<<WalkJob" << endl;
    job.toStream(output);
    return output;
}

/*! @relates WalkJob
    @brief Stream insertion operator for a pointer to WalkJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const WalkJob* job)
{
    debug << "<<WalkJob" << endl;
    if (job != NULL)
        job->toStream(output);
    else
        output << "NULL";
    return output;
}
