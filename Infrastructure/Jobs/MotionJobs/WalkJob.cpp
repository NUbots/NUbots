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
#include "debugverbosityjobs.h"

/*! @brief Constructs a WalkJob
    @param trans_speed the translational fraction
    @param trans_direction the translational direction in radians
    @param rot_speed the rotational speed in rad/s
*/
WalkJob::WalkJob(float trans_speed, float trans_direction, float rot_speed) : MotionJob(Job::MOTION_WALK)
{
    m_job_time = 0;
    m_translation_speed = trans_speed;
    m_direction = trans_direction;
    m_rotation_speed = rot_speed;
}

/*! @brief Constructs a WalkJob from stream data
    @param speed the speed for the walk job [x (cm/s), y (cm/s), theta (rad/s)]
 */
WalkJob::WalkJob(std::istream& input) : MotionJob(Job::MOTION_WALK)
{
    m_job_time = 0;
    float floatBuffer;
    input.read(reinterpret_cast<char*> (&floatBuffer), sizeof(floatBuffer));
    m_translation_speed = floatBuffer;
    input.read(reinterpret_cast<char*> (&floatBuffer), sizeof(floatBuffer));
    m_direction = floatBuffer;
    input.read(reinterpret_cast<char*> (&floatBuffer), sizeof(floatBuffer));
    m_rotation_speed = floatBuffer;
}

/*! @brief WalkJob destructor
 */
WalkJob::~WalkJob()
{
}

/*! @brief Returns the translation speed fraction (-1 to 1)
    @return the fraction of maximum translation speed
 */
float WalkJob::getTranslationSpeed()
{
    return m_translation_speed;
}

/*! @brief Returns the translation direction of the walk job
    @return the direction in radians
 */
float WalkJob::getDirection()
{
    return m_direction;
}

/*! @brief Returns the rotation speed of the walk job
    @return the rotational speed in radians per second
 */
float WalkJob::getRotationSpeed()
{
    return m_rotation_speed;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void WalkJob::summaryTo(std::ostream& output)
{
    output << "WalkJob: " << m_job_time << " " << m_translation_speed << " " << m_direction << " " << m_rotation_speed << std::endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void WalkJob::csvTo(std::ostream& output)
{
    output << "WalkJob: " << m_job_time << ", " << m_translation_speed << ", " << m_direction << ", " << m_rotation_speed << std::endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void WalkJob::toStream(std::ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 2
        debug << "WalkJob::toStream" << std::endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // Then we write WalkJob specific data
    output.write((char*) &m_translation_speed, sizeof(m_translation_speed));
    output.write((char*) &m_direction, sizeof(m_direction));
    output.write((char*) &m_rotation_speed, sizeof(m_rotation_speed));
}

/*! @relates WalkJob
    @brief Stream insertion operator for a WalkJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const WalkJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<WalkJob" << std::endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates WalkJob
    @brief Stream insertion operator for a pointer to WalkJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const WalkJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<WalkJob" << std::endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}
