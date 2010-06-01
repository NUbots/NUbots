/*! @file MotionFreezeJob.cpp
    @brief Implementation of MotionFreezeJob class

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

#include "MotionFreezeJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a MotionFreezeJob to be executed immediately
 */
MotionFreezeJob::MotionFreezeJob() : MotionJob(Job::MOTION_FREEZE)
{
}

/*! @brief MotionFreezeJob destructor
 */
MotionFreezeJob::~MotionFreezeJob()
{
}

/*! @brief Prints a human-readable summary to the stream
    @param output the stream to be written to
 */
void MotionFreezeJob::summaryTo(ostream& output)
{
    output << "MotionFreezeJob." << endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void MotionFreezeJob::csvTo(ostream& output)
{
    output << "MotionFreezeJob." << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void MotionFreezeJob::toStream(ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "MotionFreezeJob::toStream" << endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // there is no data introduced at this level
}

/*! @relates MotionFreezeJob
    @brief Stream insertion operator for a MotionFreezeJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const MotionFreezeJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 0
        debug << "<<MotionFreezeJob" << endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates MotionFreezeJob
    @brief Stream insertion operator for a pointer to MotionFreezeJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const MotionFreezeJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 0
        debug << "<<MotionFreezeJob" << endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}
