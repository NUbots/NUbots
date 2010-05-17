/*! @file HeadPanJob.cpp
    @brief Implementation of HeadPanJob class

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

#include "HeadPanJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a HeadPanJob
    @param pantype the type of pan (Ball, BallAndLocalisation, Localisation)
 */
HeadPanJob::HeadPanJob(head_pan_t pantype) : MotionJob(Job::MOTION_PAN)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::HeadPanJob()" << endl;
    #endif
    m_job_time = 0;
    m_pan_type = pantype;
    #if DEBUG_JOBS_VERBOSITY > 0
        summaryTo(debug);
    #endif
}

/*! @brief Constructs a HeadPanJob from stream data
    @param input the stream from which to read the job specific data
 */
HeadPanJob::HeadPanJob(istream& input) : MotionJob(Job::MOTION_PAN)
{
    m_job_time = 0;
    // Temporary read buffers
    head_pan_t hptBuffer;
    
    // read in the pan type
    input.read(reinterpret_cast<char*>(&hptBuffer), sizeof(hptBuffer));
    m_pan_type = hptBuffer;
    
    #if DEBUG_JOBS_VERBOSITY > 0
        summaryTo(debug);
    #endif
}

/*! @brief HeadPanJob destructor
 */
HeadPanJob::~HeadPanJob()
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::~HeadPanJob()" << endl;
    #endif
}

/*! @brief Returns the type of pan
 */
HeadPanJob::head_pan_t HeadPanJob::getPanType()
{
    return m_pan_type;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void HeadPanJob::summaryTo(ostream& output)
{
    output << "HeadPanJob: ";
    if (m_pan_type == Ball)
        output << "Ball";
    else if (m_pan_type == BallAndLocalisation)
        output << "BallAndLocalisation";
    else
        output << "Localisation";
    output << endl;
    
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void HeadPanJob::csvTo(ostream& output)
{
    output << "HeadPanJob: ";
    if (m_pan_type == Ball)
        output << "Ball";
    else if (m_pan_type == BallAndLocalisation)
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
void HeadPanJob::toStream(ostream& output) const
{
#if DEBUG_JOBS_VERBOSITY > 1
    debug << "HeadPanJob::toStream" << endl;
#endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
                                            // Then we write HeadPanJob specific data
    output.write((char*) &m_pan_type, sizeof(m_pan_type));
}

/*! @relates HeadPanJob
    @brief Stream insertion operator for a HeadPanJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadPanJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::operator<<" << endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates HeadPanJob
    @brief Stream insertion operator for a pointer to HeadPanJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadPanJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadPanJob::operator<<" << endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}

