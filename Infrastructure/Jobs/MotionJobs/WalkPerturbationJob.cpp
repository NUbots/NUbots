/*! @file WalkPerturbationJob.cpp
    @brief Implementation of WalkPerturbationJob class

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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

#include "WalkPerturbationJob.h"

#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a WalkPerturbationJob
    @param magnitude the magnitude of the perturbation (0 to 100)
    @param direction the direction of the perturbation in radians with 0 being forward, and positive to the left.
 */
WalkPerturbationJob::WalkPerturbationJob(float magnitude, float direction) : MotionJob(Job::MOTION_WALK_PERTURBATION)
{
    m_magnitude = magnitude;
    m_direction = direction;
}

/*! @brief Constructs a WalkParameterJob from stream data
    @param walkparameters the walk parameters associated with the job
 */
WalkPerturbationJob::WalkPerturbationJob(std::istream& input) : MotionJob(Job::MOTION_WALK_PARAMETERS)
{
    m_job_time = 0;
    float floatBuffer;
    input.read(reinterpret_cast<char*> (&floatBuffer), sizeof(floatBuffer));
    m_magnitude = floatBuffer;
    input.read(reinterpret_cast<char*> (&floatBuffer), sizeof(floatBuffer));
    m_direction = floatBuffer;
}

/*! @brief WalkPerturbationJob destructor
 */
WalkPerturbationJob::~WalkPerturbationJob()
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "WalkPerturbationJob::~WalkPerturbationJob()" << std::endl;
    #endif
}

/*! @brief Gets the walk parameters of the job
    @return returns the magnitude
 */
float WalkPerturbationJob::getMagnitude()
{
    return m_magnitude;
}

/*! @brief Gets the walk parameters of the job
    @return returns the direction
 */
float WalkPerturbationJob::getDirection()
{
    return m_direction;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void WalkPerturbationJob::summaryTo(std::ostream& output)
{
    output << "WalkPerturbationJob: " << m_magnitude << " " << m_direction << std::endl;;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void WalkPerturbationJob::csvTo(std::ostream& output)
{
    output << "WalkPerturbationJob, " << m_job_time << ", " << m_magnitude << ", " << m_direction << std::endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void WalkPerturbationJob::toStream(std::ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 2
        debug << "WalkPerturbationJob::toStream" << std::endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
                                            // Then we write WalkPerturbationJob specific data
    output.write((char*) &m_magnitude, sizeof(m_magnitude));
    output.write((char*) &m_direction, sizeof(m_direction));
}

/*! @relates WalkPerturbationJob
    @brief Stream insertion operator for a WalkPerturbationJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const WalkPerturbationJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<WalkPerturbationJob" << std::endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates WalkPerturbationJob
    @brief Stream insertion operator for a pointer to WalkPerturbationJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const WalkPerturbationJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<WalkPerturbationJob" << std::endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}
