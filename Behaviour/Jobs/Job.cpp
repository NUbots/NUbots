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

ostream& Job::operator<< (ostream& output)
{
    return output;
}

istream& Job::operator>> (istream& input)
{
    return input;
}

