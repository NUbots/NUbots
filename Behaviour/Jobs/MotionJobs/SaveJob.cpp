/*! @file SaveJob.cpp
    @brief Implementation of SaveJob class

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

#include "SaveJob.h"
#include "debug.h"

/*! @brief Constructs a SaveJob at the given position and time
    @param time the time in ms to perform the save
    @param position the position at which to perform the save
 */
SaveJob::SaveJob(double time, const vector<float>& position) : MotionJob(Job::MOTION_SAVE)
{
    m_job_time = time;     
    m_save_position = position;
}


/*! @brief Constructs a SaveJob from stream data
    @param time the time in ms to perform the save
    @param input the stream from which to read the job specific data
 */
SaveJob::SaveJob(double time, istream& input) : MotionJob(Job::MOTION_SAVE)
{
    m_job_time = time;

    // Temporary read buffers.
    unsigned int uintBuffer;
    float floatBuffer;

    // read in the save_position size
    input.read(reinterpret_cast<char*>(&uintBuffer), sizeof(unsigned int));
    unsigned int m_save_position_size = uintBuffer;
    
    // read in the save position vector
    m_save_position = vector<float>(m_save_position_size, 0);
    for (unsigned int i=0; i<m_save_position_size; i++)
    {
        input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
        m_save_position[i] = floatBuffer;
    }
}

/*! @brief WalkJob destructor
 */
SaveJob::~SaveJob()
{
    m_save_position.clear();
}

/*! @brief Sets the position for the save to point
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
    
    @param newposition the new position for the save job [x(cm), y(cm), theta(rad)]
 */
void SaveJob::setPosition(double time, const vector<float>& newposition)
{
    m_job_time = time;
    m_save_position = newposition;
}

/*! @brief Gets the position of the save job
    @param position parameter that will be updated with the point we want to save
 */
void SaveJob::getPosition(double& time, vector<float>& position)
{
    time = m_job_time;
    position = m_save_position;
}

/*! @brief Prints a human-readable summary to the stream
    @param output the stream to be written to
 */
void SaveJob::summaryTo(ostream& output)
{
    output << "SaveJob: " << m_job_time << " (";
    for (unsigned int i=0; i<m_save_position.size(); i++)
        output << m_save_position[i] << ",";
    output << ")" << endl;
}

/*! @brief Prints a csv version to the stream
    @param output the stream to be written to
 */
void SaveJob::csvTo(ostream& output)
{
    output << "SaveJob, " << m_job_time << ", ";
    for (unsigned int i=0; i<m_save_position.size(); i++)
        output << m_save_position[i] << ", ";
    output << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level
 
    @param output the stream to write the job to
 */
void SaveJob::toStream(ostream& output) const
{
    debug << "SaveJob::toStream" << endl;
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // Then we write SaveJob specific data
    unsigned int m_save_position_size = m_save_position.size();
    output.write((char*) &m_save_position_size, sizeof(m_save_position_size));
    for (unsigned int i=0; i<m_save_position_size; i++)
        output.write((char*) &m_save_position[i], sizeof(m_save_position[i]));
}

/*! @relates SaveJob
    @brief Stream insertion operator for a SaveJob
 
    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const SaveJob& job)
{
    debug << "<<SaveJob" << endl;
    job.toStream(output);
    return output;
}

/*! @relates SaveJob
    @brief Stream insertion operator for a pointer to SaveJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const SaveJob* job)
{
    debug << "<<SaveJob" << endl;
    if (job != NULL)
        job->toStream(output);
    else
        output << "NULL";
    return output;
}

