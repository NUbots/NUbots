/*! @file WalkParametersJob.cpp
    @brief Implementation of WalkParametersJob class

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

#include "WalkParametersJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a WalkParametersJob
    @param walkparameters the walk parameters associated with the job
 */
WalkParametersJob::WalkParametersJob(const WalkParameters& walkparameters) : MotionJob(Job::MOTION_WALK_PARAMETERS)
{
    m_walk_parameters = walkparameters;
    m_job_time = 0;         // we always want the walk parameters to change *now*
}

/*! @brief Constructs a WalkParameterJob from stream data
    @param walkparameters the walk parameters associated with the job
 */
WalkParametersJob::WalkParametersJob(std::istream& input) : MotionJob(Job::MOTION_WALK_PARAMETERS)
{
    m_job_time = 0;
    input >> m_walk_parameters;
}

/*! @brief WalkParametersJob destructor
 */
WalkParametersJob::~WalkParametersJob()
{
    #if DEBUG_JOBS_VERBOSITY > 0
        debug << "WalkParametersJob::~WalkParametersJob()" << std::endl;
    #endif
}

/*! @brief Sets the walk parameters of the job.
 
    You should only need to use this function if you are recycling the one job 
    (ie. I provide this function if you are worried about creating a new job every time)
    
    @param walkparameters the walk parameters associated with the job
 */
void WalkParametersJob::setWalkParameters(const WalkParameters& walkparameters)
{
    m_walk_parameters = walkparameters;
}

/*! @brief Gets the walk parameters of the job
    @param walkparameters the walk parameters associated with the job
 */
void WalkParametersJob::getWalkParameters(WalkParameters& walkparameters)
{
    walkparameters = m_walk_parameters;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void WalkParametersJob::summaryTo(std::ostream& output)
{
    output << "WalkParametersJob: ";
    m_walk_parameters.summaryTo(output);
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void WalkParametersJob::csvTo(std::ostream& output)
{
    output << "WalkParametersJob, " << m_job_time << ", ";
    m_walk_parameters.csvTo(output);
    output << std::endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void WalkParametersJob::toStream(std::ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 2
        debug << "WalkParametersJob::toStream" << std::endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
                                            // Then we write WalkParametersJob specific data
    output << m_walk_parameters;
}

/*! @relates WalkParametersJob
    @brief Stream insertion operator for a WalkParametersJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const WalkParametersJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<WalkParametersJob" << std::endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates WalkParametersJob
    @brief Stream insertion operator for a pointer to WalkParametersJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const WalkParametersJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<WalkParametersJob" << std::endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}
