/*! @file MotionKillJob.cpp
    @brief Implementation of MotionKillJob class

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

#include "MotionKillJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a MotionKillJob to be executed immediately
 */
MotionKillJob::MotionKillJob() : MotionJob(Job::MOTION_KILL)
{
}

/*! @brief MotionKillJob destructor
 */
MotionKillJob::~MotionKillJob()
{
}

/*! @brief Prints a human-readable summary to the stream
    @param output the stream to be written to
 */
void MotionKillJob::summaryTo(std::ostream& output)
{
    output << "MotionKillJob." << std::endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void MotionKillJob::csvTo(std::ostream& output)
{
    output << "MotionKillJob." << std::endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void MotionKillJob::toStream(std::ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "MotionKillJob::toStream" << std::endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // there is no data introduced at this level
}

/*! @relates MotionKillJob
    @brief Stream insertion operator for a MotionKillJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const MotionKillJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 0
        debug << "<<MotionKillJob" << std::endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates MotionKillJob
    @brief Stream insertion operator for a pointer to MotionKillJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const MotionKillJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 0
        debug << "<<MotionKillJob" << std::endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}
