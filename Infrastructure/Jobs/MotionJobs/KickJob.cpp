/*! @file KickJob.cpp
    @brief Partial implementation of KickJob class

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

#include "KickJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a KickJob
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
KickJob::KickJob(double time, const std::vector<float>& kickposition, const std::vector<float>& kicktarget) : MotionJob(Job::MOTION_KICK)
{
    m_job_time = time;
    m_kick_position = kickposition;
    m_kick_target = kicktarget;         
}

/*! @brief Constructs a KickJob from stream data
    @param time the time in ms to perform the kick
    @param input the stream from which to read the job specific data
 */
KickJob::KickJob(double time, std::istream& input) : MotionJob(Job::MOTION_KICK)
{
    m_job_time = time;

    // Temporary read buffers
    unsigned int uintBuffer;
    float floatBuffer;

    // read in the kick position size
    input.read(reinterpret_cast<char*>(&uintBuffer), sizeof(unsigned int));
    unsigned int m_kick_position_size = uintBuffer;
    
    // read in the kick position std::vector
    m_kick_position = std::vector<float>(m_kick_position_size, 0);
    for (unsigned int i=0; i<m_kick_position_size; i++)
    {
        input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
        m_kick_position[i] = floatBuffer;
    }
    
    // read in the kick target size
    input.read(reinterpret_cast<char*>(&uintBuffer), sizeof(unsigned int));
    unsigned int m_kick_target_size = uintBuffer;
    
    // read in the kick position std::vector
    m_kick_target = std::vector<float>(m_kick_target_size, 0);
    for (unsigned int i=0; i<m_kick_target_size; i++)
    {
        input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(float));
        m_kick_target[i] = floatBuffer;
    }
}

/*! @brief KickJob destructor
 */
KickJob::~KickJob()
{
    m_kick_position.clear();
    m_kick_target.clear();
}

/*! @brief Sets the kick time, position and target
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
void KickJob::setKick(double time, const std::vector<float>& kickposition, const std::vector<float>& kicktarget)
{
    m_job_time = time;
    m_kick_position = kickposition;
    m_kick_target = kicktarget;
}

/*! @brief Sets the kick time and position
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
 */
void KickJob::setKickPosition(double time, const std::vector<float>& kickposition)
{
    m_job_time = time;
    m_kick_position = kickposition;
}

/*! @brief Sets the kick target
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
void KickJob::setKickTarget(const std::vector<float>& kicktarget)
{
    m_kick_target = kicktarget;
}

/*! @brief Gets the kick time, position and target
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
void KickJob::getKick(double& time, std::vector<float>& kickposition, std::vector<float>& kicktarget)
{
    time = m_job_time;
    kickposition = m_kick_position;
    kicktarget = m_kick_target;
}

/*! @brief Gets the kick time and position
    @param time the time you want to kick
    @param kickposition the point you want to kick (hopefully the position of the ball) [x(cm), y(cm)]
 */
void KickJob::getKickPosition(double& time, std::vector<float>& kickposition)
{
    time = m_job_time;
    kickposition = m_kick_position;
}

/*! @brief Gets the kick target
    @param kicktarget the point you want to kick the ball at (probably the goal) from your current position [x(cm), y(cm)]
 */
void KickJob::getKickTarget(std::vector<float>& kicktarget)
{
    kicktarget = m_kick_target;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void KickJob::summaryTo(std::ostream& output)
{
    output << "KickJob: " << m_job_time << " (";
    for (unsigned int i=0; i<m_kick_position.size(); i++)
        output << m_kick_position[i] << ",";
    output << ")-->(";
    for (unsigned int i=0; i<m_kick_target.size(); i++)
        output << m_kick_target[i] << ",";
    output << ")" << std::endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void KickJob::csvTo(std::ostream& output)
{
    output << "KickJob: " << m_job_time << "(";
    for (unsigned int i=0; i<m_kick_position.size(); i++)
        output << m_kick_position[i] << ", ";
    for (unsigned int i=0; i<m_kick_target.size(); i++)
        output << m_kick_target[i] << ", ";
    output << std::endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
 This function calls its parents versions of the toStream, each parent
 writes the members introduced at that level
 
 @param output the stream to write the job to
 */
void KickJob::toStream(std::ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 2
        debug << "KickJob::toStream" << std::endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // Then we write KickJob specific data
    unsigned int m_kick_position_size = m_kick_position.size();
    output.write((char*) &m_kick_position_size, sizeof(m_kick_position_size));
    for (unsigned int i=0; i<m_kick_position_size; i++)
        output.write((char*) &m_kick_position[i], sizeof(m_kick_position[i]));
    unsigned int m_kick_target_size = m_kick_target.size();
    output.write((char*) &m_kick_target_size, sizeof(m_kick_target_size));
    for (unsigned int i=0; i<m_kick_target_size; i++)
        output.write((char*) &m_kick_target[i], sizeof(m_kick_target[i]));
}

/*! @relates KickJob
    @brief Stream insertion operator for a KickJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const KickJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<KickJob" << std::endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates KickJob
    @brief Stream insertion operator for a pointer to KickJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const KickJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<KickJob" << std::endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}
