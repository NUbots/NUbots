/*! @file BlockJob.cpp
    @brief Implementation of BlockJob class

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

#include "BlockJob.h"
#include "debug.h"

/*! @brief Constructs a BlockJob at the given position and time
    @param time the time in ms to perform the save
    @param position the position at which to perform the block
 */
BlockJob::BlockJob(double time, const vector<float>& position) : MotionJob(Job::MOTION_BLOCK)
{
    m_job_time = time;     
    m_block_position = position;
}

/*! @brief Constructs a BlockJob from stream data
    @param time the time in ms to perform the save
    @param input the stream from which to read the job specific data
 */
BlockJob::BlockJob(double time, istream& input) : MotionJob(Job::MOTION_BLOCK)
{
    m_job_time = time;
    char buffer[1024];
    // read in the save_position size
    input.read(buffer, sizeof(unsigned int));
    unsigned int m_block_position_size = *reinterpret_cast<unsigned int*>(buffer);
    
    // read in the save position vector
    m_block_position = vector<float>(m_block_position_size, 0);
    for (unsigned int i=0; i<m_block_position_size; i++)
    {
        input.read(buffer, sizeof(float));
        m_block_position[i] = *reinterpret_cast<float*>(buffer);
    }
}

/*! @brief WalkJob destructor
 */
BlockJob::~BlockJob()
{
    m_block_position.clear();
}

/*! @brief Sets the position for the block to point
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
    
    @param newposition the new position for the block job [x(cm), y(cm), theta(rad)]
 */
void BlockJob::setPosition(double time, const vector<float>& newposition)
{
    m_job_time = time;
    m_block_position = newposition;
}

/*! @brief Gets the position of the block job
    @param position parameter that will be updated with the point we want to block
 */
void BlockJob::getPosition(double& time, vector<float>& position)
{
    time = m_job_time;
    position = m_block_position;
}

/*! @brief Prints a human-readable summary to the stream
    @param output the stream to be written to
 */
void BlockJob::summaryTo(ostream& output)
{
    output << "BlockJob: " << m_job_time << " (";
    for (unsigned int i=0; i<m_block_position.size(); i++)
        output << m_block_position[i] << ",";
    output << ")" << endl;
}

/*! @brief Prints a csv version to the stream
    @param output the stream to be written to
 */
void BlockJob::csvTo(ostream& output)
{
    output << "BlockJob, " << m_job_time << ", ";
    for (unsigned int i=0; i<m_block_position.size(); i++)
        output << m_block_position[i] << ", ";
    output << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void BlockJob::toStream(ostream& output) const
{
    debug << "BlockJob::toStream" << endl;
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // Then we write BlockJob specific data
    unsigned int m_block_position_size = m_block_position.size();
    output.write((char*) &m_block_position_size, sizeof(m_block_position_size));
    for (unsigned int i=0; i<m_block_position_size; i++)
        output.write((char*) &m_block_position[i], sizeof(m_block_position[i]));
}

/*! @relates BlockJob
    @brief Stream insertion operator for a BlockJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const BlockJob& job)
{
    debug << "<<BlockJob" << endl;
    job.toStream(output);
    return output;
}

/*! @relates BlockJob
    @brief Stream insertion operator for a pointer to BlockJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const BlockJob* job)
{
    debug << "<<BlockJob" << endl;
    if (job != NULL)
        job->toStream(output);
    else
        output << "NULL";
    return output;
}

