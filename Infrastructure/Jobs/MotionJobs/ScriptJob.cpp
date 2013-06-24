/*! @file ScriptJob.cpp
    @brief Implementation of ScriptJob class

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

#include "ScriptJob.h"

#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a ScriptJob at the given position and time
    @param time the time in ms to perform the save
    @param position the position at which to perform the save
 */
// ScriptJob::ScriptJob(double time, const MotionScript2013* script) : MotionJob(Job::MOTION_SCRIPT)
// {
//     m_job_time = time;     
//     m_script = script;
//     m_name = m_script->getName();
// }

/*! @brief Constructs a ScriptJob at the given time
    @param time the time in ms to perform the save
    @param name the name of the script to play  
 */
ScriptJob::ScriptJob(double time, const std::string& name) : MotionJob(Job::MOTION_SCRIPT)
{
    m_job_time = time;
    m_name = name;
}

/*! @brief Constructs a ScriptJob from stream data
    @param time the time in ms to perform the save
    @param input the stream from which to read the job specific data
 */
ScriptJob::ScriptJob(double time, std::istream& input) : MotionJob(Job::MOTION_SCRIPT)
{
    m_job_time = time;
    input >> m_name; 
    //input >> m_script;
}

/*! @brief WalkJob destructor
 */
ScriptJob::~ScriptJob()
{
}

/*! @brief Gets the script associated with the script job
    @param time will be updated with the time to play the script
    @param script the motion script to be played
 */
void ScriptJob::getScript(double& time, MotionScript2013* script)
{
    time = m_job_time;
    // if (not m_script.isValid())
    //     script = MotionScript2013::LoadFromConfigSystem(m_name);
    // else
    script = m_script;
}

/*! @brief Returns the name of the associated script
 */
std::string& ScriptJob::getName()
{
    return m_name;
}

/*! @brief Prints a human-readable summary to the stream
    @param output the stream to be written to
 */
void ScriptJob::summaryTo(std::ostream& output)
{
    output << "ScriptJob: " << m_job_time << " " << m_name << std::endl;;
}

/*! @brief Prints a csv version to the stream
    @param output the stream to be written to
 */
void ScriptJob::csvTo(std::ostream& output)
{
    output << "ScriptJob, " << m_job_time << ", " << m_name << std::endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level
 
    @param output the stream to write the job to
 */
void ScriptJob::toStream(std::ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 2
        debug << "ScriptJob::toStream" << std::endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // Then we write ScriptJob specific data
    output << m_name;
    // output << m_script;
}

/*! @relates ScriptJob
    @brief Stream insertion operator for a ScriptJob
 
    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const ScriptJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<ScriptJob" << std::endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates ScriptJob
    @brief Stream insertion operator for a pointer to ScriptJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const ScriptJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "<<ScriptJob" << std::endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}

