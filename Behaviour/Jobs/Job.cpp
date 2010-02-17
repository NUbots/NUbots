/*! @file Job.cpp
    @brief Partial implementation of base job class

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

#include "Job.h"
#include "../Jobs.h"
#include "debug.h"

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

/*! @brief Returns the unix timestamp of when the job was created
 */
long double Job::getTimeStamp()
{
    return m_timestamp;
}

void Job::summaryTo(ostream& output)
{
}

void Job::csvTo(ostream& output)
{
}

ostream& operator<<(ostream& output, const Job& job)
{
    debug << "Job<<" << endl;
    
    output << static_cast<unsigned int>(job.m_job_type) << " " << static_cast<unsigned int>(job.m_job_id) << " ";
    output.write((char*) &job.m_job_time, sizeof(job.m_job_time));
    output.write((char*) &job.m_timestamp, sizeof(job.m_timestamp));
    
    return output;
}

istream& operator>>(istream& input, Job& job)
{
    // This is a very very very hard function to write!
    debug << "Job>>" << endl;
    
    Job::job_type_t jobtype;    // we need to create a job of the correct type!
    Job::job_id_t jobid;        // we need to create a job of the correct id!
    
    unsigned int tempint = 0;
    
    // Read in the job's type and id
    input >> tempint;
    jobtype = static_cast<Job::job_type_t>(tempint);
    input >> tempint;
    jobid = static_cast<Job::job_id_t>(tempint);
    
    debug << jobtype << " " << jobid << endl;
    // Now create a new job of that type
    switch (jobid) 
    {
        case Job::MOTION_SAVE:
            /*job = SaveJob(0, vector<float>(3,0));
            input >> static_cast<SaveJob> (job);*/
            break;
        default:
            break;
    }
    
    return input;
}

