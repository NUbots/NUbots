/*! @file Job.cpp
    @brief Partial implementation of base job class

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

#include "Job.h"
#include "../Jobs.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Job constructor
 */
Job::Job(job_type_t jobtype, job_id_t jobid) : m_job_type(jobtype), m_job_id(jobid)
{
}

/*! @brief Job destructor
 */
Job::~Job()
{
    // I think everything will cleaning delete itself.
}

/*! @brief Get the job's type
    @returns the job's type
 */
Job::job_type_t Job::getType()
{
    return m_job_type;
}

/*! @brief Get the job's id
    @returns the job's id
 */
Job::job_id_t Job::getID()
{
    return m_job_id;
}

/*! @brief Get the time at which the job is to be completed
    @returns the job time
 */
double Job::getTime()
{
    return m_job_time;
}

/*! @brief A helper function to ease writing Job objects to stream.
 
    The idea behind this virtual function is for each child Job to write its
    own data to the stream. Each class level writes the members introduced at that level,
    so that each child's implementation of toStream looks like:
        @code
        Job::toStream(output);
        MotionJob::toStream(output);
        WalkJob::toStream(output);
        WalkToPointJob::toStream(output);
        @endcode
    and so on. I did it this way so I didn't have to write the implementation for the middle levels
    in every child.
 
    @param output the stream to write the job to
 */
void Job::toStream(ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "Job::toStream()" << endl;
    #endif
    
    output << static_cast<unsigned int>(m_job_type) << " " << static_cast<unsigned int>(m_job_id) << " ";
    output.write((char*) &m_job_time, sizeof(m_job_time));
}

/*! @relates Job
    @brief Stream insertion operator for Job.
 
    This operator calls the protected virtual member toStream(output). As toStream is virtual
    the correct toStream function will be called for all types of jobs. Furthermore,
    toStream functions for abstract classes write members introduced at that level of the
    hierachy.
 
    @param output the stream to put the job in
    @param job the job to put in the stream
 */
ostream& operator<<(ostream& output, const Job& job)
{
    job.toStream(output);
    return output;
}

/*! @relates Job
    @brief Stream insertion operator for pointer to job
 
    This operator is overloaded so that it writes the object pointed to by job,
    not the value of the pointer itself.
 
    @param output the stream to put the job in
    @param job the pointer to the job to put in the stream
 */
ostream& operator<<(ostream& output, const Job* job)
{
    if (job != NULL)
        job->toStream(output);
    return output;
}

/*! @relates Job
    @brief Stream extraction operator for Job
 
    This operator 
 
    @param input the stream in which the job is stored
    @param job a pointer to the pointer to the job to store the job extracted from the stream
           (it is done this way to avoid using the assignment operator which I haven't written yet)
 
    @attention This operator needs to be updated when you want to stream a new type of Job.
               You need to add a
                    @code
                    case Job::_JOB_ID_:
                        *job = new NewTypeJob(jobtime, input);
                        break;
                    @endcode
               there isn't a way around this (yet).
 */
istream& operator>>(istream& input, Job** job)
{
#if DEBUG_JOBS_VERBOSITY > 4
    debug << ">>Job**" << endl;
#endif
    
    unsigned int tempuint = 0;

    // Buffer for reading
    char charBuffer;
    double doubleBuffer;

    Job::job_type_t jobtype;
    Job::job_id_t jobid;
    double jobtime;
    
    // Read the type and id
    input >> tempuint;
    jobtype = static_cast<Job::job_type_t>(tempuint);
    input >> tempuint;
    jobid = static_cast<Job::job_id_t>(tempuint);

    // Also read in the time (because it was written at the Job level)
    input.read(&charBuffer, sizeof(char));           // skip the white space!
    input.read(reinterpret_cast<char*>(&doubleBuffer), sizeof(double));
    jobtime = doubleBuffer;
    
    // Now that we have the id (and the type) create a new Job of the correct concrete type
    switch (jobid) 
    {
        case Job::MOTION_WALK_TO_POINT:
            *job = new WalkToPointJob(jobtime, input);
            break;
        case Job::MOTION_WALK:
            *job = new WalkJob(input);
            break;
        case Job::MOTION_WALK_PARAMETERS:
            *job = new WalkParametersJob(input);
            break;
        case Job::MOTION_KICK:
            *job = new KickJob(jobtime, input);
            break;
        case Job::MOTION_BLOCK:
            *job = new BlockJob(jobtime, input);
            break;
        case Job::MOTION_SAVE:
            *job = new SaveJob(jobtime, input);
            break;
        case Job::MOTION_HEAD:
            *job = new HeadJob(jobtime, input);
            break;
        case Job::MOTION_NOD:
            *job = new HeadNodJob(input);
            break;
        case Job::MOTION_PAN:
            *job = new HeadPanJob(input);
            break;
        case Job::CAMERA_CHANGE_SETTINGS:
            *job = new ChangeCameraSettingsJob(input);
            break;
        case Job::VISION_SAVE_IMAGES:
            *job = new SaveImagesJob(input);
            break;
        case Job::SOUND_FILE:
            *job = new SoundJob(jobtime, input);
            break;
        default:
            errorlog << "Job::operator>>. UNKNOWN JOBID: " << jobid << ". Your stream might never recover :(" << endl;
            break;
    }    
#if DEBUG_JOBS_VERBOSITY > 4
    if (*job != NULL)
        (*job)->summaryTo(debug);
#endif
    return input;
}

