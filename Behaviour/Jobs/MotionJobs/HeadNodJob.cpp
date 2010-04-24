/*! @file HeadNodJob.cpp
    @brief Implementation of HeadNodJob class

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

#include "HeadNodJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a HeadNodJob
 
    @param period the new nod period in milliseconds
    @param centre the centre head position about which we will nod [yaw (rad), pitch (rad), roll (rad)]
    @param limits the lower and upper angle limits for the nod (rad)
 */
HeadNodJob::HeadNodJob(head_nod_t nodtype) : MotionJob(Job::MOTION_NOD)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadNodJob::HeadNodJob()" << endl;
    #endif
        m_job_time = 0;
        m_nod_type = nodtype;
    #if DEBUG_JOBS_VERBOSITY > 0
        summaryTo(debug);
    #endif
}

/*! @brief Constructs a HeadNodJob from stream data
    @param input the stream from which to read the job specific data
 */
HeadNodJob::HeadNodJob(istream& input) : MotionJob(Job::MOTION_NOD)
{
    m_job_time = 0;
    // Temporary read buffers
    head_nod_t hntBuffer;
    
    // read in the pan type
    input.read(reinterpret_cast<char*>(&hntBuffer), sizeof(hntBuffer));
    m_nod_type = hntBuffer;
    
#if DEBUG_JOBS_VERBOSITY > 0
    summaryTo(debug);
#endif
}

/*! @brief HeadNodJob destructor
 */
HeadNodJob::~HeadNodJob()
{
}

/* @ brief Returns the type of nod
 */
HeadNodJob::head_nod_t HeadNodJob::getNodType()
{
    return m_nod_type;
}

/*! @brief Prints a human-readable summary to the stream
    @param output the stream to be written to
 */
void HeadNodJob::summaryTo(ostream& output)
{
    output << "HeadNodJob: ";
    if (m_nod_type == Ball)
        output << "Ball";
    else if (m_nod_type == BallAndLocalisation)
        output << "BallAndLocalisation";
    else
        output << "Localisation";
    output << endl;
}

/*! @brief Prints a csv version to the stream
    @param output the stream to be written to
 */
void HeadNodJob::csvTo(ostream& output)
{
    output << "HeadNodJob: ";
    if (m_nod_type == Ball)
        output << "Ball";
    else if (m_nod_type == BallAndLocalisation)
        output << "BallAndLocalisation";
    else
        output << "Localisation";
    output << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void HeadNodJob::toStream(ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::toStream" << endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
                                            // Then we write HeadPanJob specific data
    output.write((char*) &m_nod_type, sizeof(m_nod_type));
}

/*! @relates HeadNodJob
    @brief Stream insertion operator for a HeadNodJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadNodJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadNodJob::operator<<" << endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates HeadNodJob
    @brief Stream insertion operator for a pointer to HeadNodJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadNodJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadNodJob::operator<<" << endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}
