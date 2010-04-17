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
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a HeadJob at the given position and time
    @param time the time in ms to perform the move
    @param position the head will move to this position
 */
HeadJob::HeadJob(double time, const vector<float>& position) : MotionJob(Job::MOTION_HEAD)
{
    setPosition(time, position);
}

/*! @brief Constructs a HeadJob for a given sequence of positions and times
    @param times the time for each position in ms
    @param positions the sequence head positions 
 
    Note that times and positions should have the same length, and that positions must be rectangular.
 */
HeadJob::HeadJob(const vector<double>& times, const vector<vector<float> >& positions) : MotionJob(Job::MOTION_HEAD)
{
    setPositions(times, positions);
}
 
/*! @brief Constructs a HeadJob from stream data
    @param time the time in ms to perform the save
    @param input the stream from which to read the job specific data
 */
HeadJob::HeadJob(double time, istream& input) : MotionJob(Job::MOTION_HEAD)
{
    m_job_time = time;

    // Temporary read buffers.
    unsigned int uintBuffer;
    float floatBuffer;
    double doubleBuffer;

    // read in the head_position size
    input.read(reinterpret_cast<char*>(&uintBuffer), sizeof(uintBuffer));
    unsigned int times_size = uintBuffer;
    
    // read in the head_position vector
    m_times = vector<double>(times_size, 0);
    m_head_positions = vector<vector<float> > (times_size, vector<float>(0,0));
    for (unsigned int i=0; i<times_size; i++)
    {
        input.read(reinterpret_cast<char*>(&doubleBuffer), sizeof(doubleBuffer));
        m_times[i] = doubleBuffer;
        input.read(reinterpret_cast<char*>(&uintBuffer), sizeof(uintBuffer));
        unsigned position_size = uintBuffer;
        for (unsigned int j=0; j<position_size; j++)
        {
            input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
            m_head_positions[i].push_back(floatBuffer);
        }
    }
}

/*! @brief WalkJob destructor
 */
HeadJob::~HeadJob()
{
    m_times.clear();
    m_head_positions.clear();
}

/*! @brief Sets the position for the head to point
    @param newposition the new position for the head job [roll, pitch, yaw]
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
 */
void HeadJob::setPosition(double time, const vector<float>& newposition)
{
    m_job_time = time;     
    m_times = vector<double> (1, time);
    m_head_positions = vector<vector<float> > (1, newposition);
}

/*! @brief Sets the sequence of positions and times
    @param times the time for each position in ms
    @param positions the sequence head positions 

    Note that times and positions should have the same length, and that positions must be rectangular.
 */
void HeadJob::setPositions(const vector<double>& times, const vector<vector<float> >& positions)
{
    if (times.size() != 0 && positions.size() != 0)
    {
        m_job_time = times[0];
        m_times = times;
        m_head_positions = positions;
    }
}

/*! @brief Gets the sequence of points for the head
    @param times the time for each point in the sequence
    @param positions the sequence of positions
 */
void HeadJob::getPositions(vector<double>& times, vector<vector<float> >& positions)
{
    times = m_times;
    positions = m_head_positions;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void HeadJob::summaryTo(ostream& output)
{
    output << "HeadJob: " << m_job_time << " ";
    for (unsigned int i=0; i<m_times.size(); i++)
    {
        output << "(" << m_times[i] << ": (";
        for (unsigned int j=0; j<m_head_positions[i].size(); j++)
        {
            output << m_head_positions[i][j] << ",";
        }
        output << ") ";
    }
    output << endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void HeadJob::csvTo(ostream& output)
{
    output << "HeadJob: " << ", ";
    for (unsigned int i=0; i<m_times.size(); i++)
    {
        output << m_times[i] << ", ";
        for (unsigned int j=0; j<m_head_positions[i].size(); j++)
        {
            output << m_head_positions[i][j] << ",";
        }
    }
    output << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void HeadJob::toStream(ostream& output) const
{
    debug << "HeadJob::toStream" << endl;
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // Then we write HeadJob specific data (m_times and m_head_positions)
    unsigned int times_size = m_times.size();
    output.write((char*) &times_size, sizeof(times_size));
    for (unsigned int i=0; i<times_size; i++)
    {
        output.write((char*) &m_times[i], sizeof(m_times[i]));
        unsigned int position_size = m_head_positions[i].size();
        output.write((char*) &position_size, sizeof(position_size));
        for (unsigned int j=0; j<position_size; j++)
            output.write((char*) &m_head_positions[i][j], sizeof(m_head_positions[i][j]));
    }
}

/*! @relates HeadJob
    @brief Stream insertion operator for a HeadJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadJob& job)
{
    debug << "<<HeadJob" << endl;
    job.toStream(output);
    return output;
}

/*! @relates HeadJob
    @brief Stream insertion operator for a pointer to HeadJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadJob* job)
{
    debug << "<<HeadJob" << endl;
    if (job != NULL)
        job->toStream(output);
    else
        output << "NULL";
    return output;
}
