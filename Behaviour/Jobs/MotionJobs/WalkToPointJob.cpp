/*! @file WalkToPointJob.cpp
    @brief Implementation of WalkToPointJob class

    @author Jason Kulk
 
 Copyright (c) 2009, 2010 Jason Kulk
 
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

#include "WalkToPointJob.h"
#include "debug.h"

/*! @brief Constructs a WalkToPointJob
    @param time the time in ms to reach the position
    @param position the position for the walk job [x(cm), y(cm), theta(rad)]
 */
WalkToPointJob::WalkToPointJob(double time, const vector<float>& position) : MotionJob(Job::MOTION_WALK_TO_POINT)
{
    m_job_time = time;     
    m_walk_position = position;
}

/*! @brief Constructs a WalkToPointJob from stream data
    @param time the time in ms to reach the position
    @param input the stream from which to read the job specific data
 */
WalkToPointJob::WalkToPointJob(double time, istream& input) : MotionJob(Job::MOTION_WALK_TO_POINT)
{
    m_job_time = time;

    // Temporary read buffers.
    unsigned int uintBuffer;
    float floatBuffer;

    // read in the m_walk_position size
    input.read(reinterpret_cast<char*>(&uintBuffer), sizeof(unsigned int));
    unsigned int m_walk_position_size = uintBuffer;
    
    // read in the m_walk_position vector
    m_walk_position = vector<float>(m_walk_position_size, 0);
    for (unsigned int i=0; i<m_walk_position_size; i++)
    {
        input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
        m_walk_position[i] = floatBuffer;
    }
}

/*! @brief WalkJob destructor
 */
WalkToPointJob::~WalkToPointJob()
{
    m_walk_position.clear();
}

/*! @brief Sets the position for the walk to point
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
    
    @param newposition the new position for the walk job [x(cm), y(cm), theta(rad)]
 */
void WalkToPointJob::setPosition(double time, const vector<float>& newposition)
{
    m_job_time = time;
    m_walk_position = newposition;
}

/*! @brief Gets the position of the walk job
    @param position parameter that will be updated with the point we want to walk to
 */
void WalkToPointJob::getPosition(double& time, vector<float>& position)
{
    time = m_job_time;
    position = m_walk_position;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void WalkToPointJob::summaryTo(ostream& output)
{
    output << "WalkToPointJob: " << m_job_time << " ";
    for (unsigned int i=0; i<m_walk_position.size(); i++)
        output << m_walk_position[i] << ",";
    output << endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void WalkToPointJob::csvTo(ostream& output)
{
    output << "WalkToPointJob, " << m_job_time << ", "; 
    for (unsigned int i=0; i<m_walk_position.size(); i++)
        output << m_walk_position[i] << ", ";
    output << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void WalkToPointJob::toStream(ostream& output) const
{
    debug << "WalkToPointJob::toStream" << endl;
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
                                            // Then we write WalkToPointJob specific data
    unsigned int m_walk_position_size = m_walk_position.size();
    output.write((char*) &m_walk_position_size, sizeof(m_walk_position_size));
    for (unsigned int i=0; i<m_walk_position_size; i++)
        output.write((char*) &m_walk_position[i], sizeof(m_walk_position[i]));
}

/*! @relates WalkToPointJob
    @brief Stream insertion operator for a WalkToPointJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const WalkToPointJob& job)
{
    debug << "<<WalkToPointJob" << endl;
    job.toStream(output);
    return output;
}

/*! @relates WalkToPointJob
    @brief Stream insertion operator for a pointer to WalkToPointJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const WalkToPointJob* job)
{
    debug << "<<WalkToPointJob" << endl;
    if (job != NULL)
        job->toStream(output);
    else
        output << "NULL";
    return output;
}
